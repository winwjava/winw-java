"""Natural-scene stereo calibration through COLMAP; no calibration target.

Requires numpy, opencv-python and pycolmap (or a COLMAP CLI). A measured baseline fixes metric scale.
Raw synchronized pairs must live in images/left and images/right.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import uuid

import numpy as np


def rotation(q):
    q = np.asarray(q, dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError("Invalid quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def angle(a, b):
    return math.degrees(math.acos(float(np.clip((np.trace(a @ b.T)-1)/2, -1, 1))))


def read_model(path):
    cameras = {}
    for line in (path / "cameras.txt").read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        p = line.split()
        cameras[int(p[0])] = (p[1], int(p[2]), int(p[3]), np.array(list(map(float, p[4:]))))
    images = {}
    # Each image pose is followed by exactly one observation line, which may be empty.
    lines = iter((path / "images.txt").read_text().splitlines())
    for line in lines:
        if not line.strip() or line.startswith("#"):
            continue
        p = line.split(maxsplit=9)
        if len(p) != 10:
            raise ValueError("Invalid COLMAP images.txt pose")
        name = p[9].replace("\\", "/")
        images[name] = (int(p[8]), rotation(list(map(float, p[1:5]))), np.array(list(map(float, p[5:8]))))
        next(lines)  # Consume observation line, including empty lines.
    return cameras, images


def camera_values(camera):
    model, width, height, p = camera
    if not np.all(np.isfinite(p)) or width < 160 or height < 120:
        raise ValueError("Invalid camera dimensions/parameters")
    if model == "SIMPLE_RADIAL" and len(p) == 4:
        f, cx, cy, k1 = p
        fx, fy, d = f, f, [k1, 0, 0, 0, 0]
    elif model == "RADIAL" and len(p) == 5:
        f, cx, cy, k1, k2 = p
        fx, fy, d = f, f, [k1, k2, 0, 0, 0]
    elif model == "OPENCV" and len(p) == 8:
        fx, fy, cx, cy, k1, k2, p1, p2 = p
        d = [k1, k2, p1, p2, 0]
    else:
        raise ValueError("Unsupported model: " + model)
    if not (.2*max(width, height) < fx < 5*max(width, height) and .2*max(width, height) < fy < 5*max(width, height)
            and .25*width < cx < .75*width and .25*height < cy < .75*height):
        raise ValueError("Degenerate intrinsics: collect richer camera motion")
    # The distortion map must remain orientation-preserving over the image.
    max_radius = math.hypot(max(cx, width-cx)/fx, max(cy, height-cy)/fy)
    for radius in np.linspace(0, max_radius, 40):
        radial = 1+d[0]*radius**2+d[1]*radius**4
        derivative = 1+3*d[0]*radius**2+5*d[1]*radius**4
        if radial < .2 or derivative < .2:
            raise ValueError("Degenerate radial distortion")
    return width, height, [fx, 0, cx, 0, fy, cy, 0, 0, 1], d


def calibrate(cameras, images, baseline, min_pairs=12):
    if not math.isfinite(baseline) or baseline <= 0:
        raise ValueError("Supply a measured positive baseline in metres")
    pairs = [(name, name.replace("left/", "right/", 1)) for name in sorted(images)
             if name.startswith("left/") and name.replace("left/", "right/", 1) in images]
    if len(pairs) < min_pairs:
        raise ValueError(f"Only {len(pairs)} reconstructed pairs; need at least {min_pairs}")
    left_ids = {images[l][0] for l, _ in pairs}
    right_ids = {images[r][0] for _, r in pairs}
    if len(left_ids) != 1 or len(right_ids) != 1 or left_ids == right_ids:
        raise ValueError("Each eye must share one separate camera model across all frames")
    left = camera_values(cameras[next(iter(left_ids))])
    right = camera_values(cameras[next(iter(right_ids))])
    if left[:2] != right[:2]:
        raise ValueError("Left/right image dimensions differ")
    rotations, translations, centers, poses = [], [], [], []
    for l, r in pairs:
        _, rl, tl = images[l]
        _, rr, tr = images[r]
        rotations.append(rr @ rl.T)
        translations.append(tr - rotations[-1] @ tl)
        centers.append(-rl.T @ tl)
        poses.append(rl)
    translations = np.asarray(translations)
    lengths = np.linalg.norm(translations, axis=1)
    length = float(np.median(lengths))
    if not np.all(np.isfinite(translations)) or length < 1e-9:
        raise ValueError("Degenerate stereo translation")
    t = np.median(translations, axis=0)
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    r = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    rotation_errors = np.array([angle(x, r) for x in rotations])
    translation_errors = np.linalg.norm(translations-t, axis=1) / length
    # Reject the whole sequence when the reconstructed rigid stereo geometry is inconsistent.
    if np.percentile(rotation_errors, 90) > 1.0 or np.percentile(translation_errors, 90) > .10:
        raise ValueError("Stereo geometry is inconsistent (>1 degree or >10%); check sync, motion, lens focus")
    spans = np.linalg.svd(np.asarray(centers)-np.mean(centers, axis=0), compute_uv=False) / math.sqrt(len(centers)) / length
    turn = max(angle(x, poses[0]) for x in poses)
    if spans[0] < 2 or spans[1] < .25 or turn < 5:
        raise ValueError("Insufficient camera excitation: translate sideways/up-down and rotate gently")
    t = t / np.linalg.norm(t) * baseline
    if t[0] >= 0 or abs(t[1]) > .3*baseline or abs(t[2]) > .3*baseline:
        raise ValueError("Not a conventional horizontal left/right rig; check eye order")
    report = {"pairs": len(pairs), "baseline_metres": baseline,
              "rotation_p90_degrees": float(np.percentile(rotation_errors, 90)),
              "translation_p90_fraction": float(np.percentile(translation_errors, 90)),
              "motion_spans_in_baselines": spans.tolist(), "rotation_span_degrees": turn,
              "method": "natural-scene COLMAP self-calibration; measured baseline fixes scale",
              "accuracy": "not a certified metric calibration; validate with independent distances"}
    return left, right, r, t, report


def write_properties(destination, result):
    left, right, r, t, report = result
    def values(x):
        return ",".join(format(float(v), ".17g") for v in np.asarray(x).ravel())
    content = ("# Natural-scene self-calibration. See companion quality report.\nrectified=false\n"
               f"width={left[0]}\nheight={left[1]}\n"
               f"left.K={values(left[2])}\nleft.D={values(left[3])}\n"
               f"right.K={values(right[2])}\nright.D={values(right[3])}\n"
               f"R={values(r)}\nT={values(t)}\n")
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Never silently replace a previously verified calibration.
    with destination.open("x", encoding="utf-8") as stream:
        stream.write(content)
    destination.with_suffix(".quality.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


def split_sequence(sequence):
    images = sequence / "images"
    names = [{p.name for p in (images / eye).glob("*.png")} for eye in ("left", "right")]
    if names[0] != names[1] or len(names[0]) < 25:
        raise ValueError("Need at least 25 synchronized raw pairs in images/left and images/right")
    ordered = sorted(names[0])
    heldout = ordered[::5]
    training = [f"{eye}/{name}" for name in ordered if name not in heldout for eye in ("left", "right")]
    return training, heldout


def validate_heldout(sequence, names, result):
    """Reserved raw pairs never used by SfM; check rectified epipolar geometry and positive depth."""
    try:
        import cv2
    except ImportError as error:
        raise ValueError("Install opencv-python to validate independent raw pairs") from error
    left, right, r, t, report = result
    kl, kr = np.array(left[2]).reshape(3, 3), np.array(right[2]).reshape(3, 3)
    dl, dr = np.array(left[3]).reshape(1, 5), np.array(right[3]).reshape(1, 5)
    rl, rr, pl, pr, _, _, _ = cv2.stereoRectify(kl, dl, kr, dr, left[:2], r, t.reshape(3, 1),
                                               flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    orb = cv2.ORB_create(5000)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    errors, successes, diagnostics = [], 0, []
    for name in names:
        a = cv2.imread(str(sequence / "images/left" / name), cv2.IMREAD_GRAYSCALE)
        b = cv2.imread(str(sequence / "images/right" / name), cv2.IMREAD_GRAYSCALE)
        if a is None or b is None or a.shape != (left[1], left[0]) or b.shape != a.shape:
            raise ValueError("Missing or mismatched validation image " + name)
        ka, da = orb.detectAndCompute(a, None)
        kb, db = orb.detectAndCompute(b, None)
        if da is None or db is None:
            diagnostics.append({"name": name, "status": "no features"})
            continue
        back = {m[0].queryIdx: m[0].trainIdx for m in matcher.knnMatch(db, da, k=2)
                if len(m) == 2 and m[0].distance < .75*m[1].distance}
        matches = [m[0] for m in matcher.knnMatch(da, db, k=2) if len(m) == 2
                   and m[0].distance < .75*m[1].distance and back.get(m[0].trainIdx) == m[0].queryIdx]
        if len(matches) < 60:
            diagnostics.append({"name": name, "status": "too few matches", "matches": len(matches)})
            continue
        a_pts = np.array([ka[m.queryIdx].pt for m in matches], dtype=float)
        b_pts = np.array([kb[m.trainIdx].pt for m in matches], dtype=float)
        cells = {(min(3, int(x*4/left[0])), min(2, int(y*3/left[1]))) for x, y in a_pts}
        if len(cells) < 8:
            diagnostics.append({"name": name, "status": "poor coverage"})
            continue
        rect_a = cv2.undistortPoints(a_pts.reshape(-1, 1, 2), kl, dl, R=rl, P=pl).reshape(-1, 2)
        rect_b = cv2.undistortPoints(b_pts.reshape(-1, 1, 2), kr, dr, R=rr, P=pr).reshape(-1, 2)
        error = np.abs(rect_a[:, 1]-rect_b[:, 1])
        disparity = rect_a[:, 0]-rect_b[:, 0]-(pl[0, 2]-pr[0, 2])
        e80 = float(np.percentile(error, 80))
        positive = float(np.mean(disparity > .25))
        passed = math.isfinite(e80) and e80 <= 1.5 and positive >= .85
        diagnostics.append({"name": name, "matches": len(matches), "vertical_p80_pixels": e80,
                            "positive_disparity_fraction": positive, "passed": passed})
        if passed:
            successes += 1
            errors.extend(error.tolist())
    validation = {"reserved_pairs": len(names), "passed_pairs": successes, "frames": diagnostics}
    # Persist diagnostic evidence even when the calibration is rejected.
    (sequence / ("validation-" + uuid.uuid4().hex + ".json")).write_text(json.dumps(validation, indent=2), encoding="utf-8")
    if successes < 3 or successes < .6*len(names):
        raise ValueError(f"Independent validation failed: only {successes}/{len(names)} pairs passed; calibration NOT exported")
    report["independent_validation"] = validation
    report["heldout_vertical_p80_pixels"] = float(np.percentile(errors, 80))


def reconstruct_python(sequence, workspace, training):
    try:
        import pycolmap
    except ImportError as error:
        raise ValueError("Install pycolmap or provide a COLMAP executable; see AUTO-CALIBRATION.md") from error
    database = workspace / "database.db"
    sparse = workspace / "sparse"
    sparse.mkdir()
    pycolmap.extract_features(database_path=str(database), image_path=str(sequence / "images"),
                             camera_mode=pycolmap.CameraMode.PER_FOLDER, image_names=training,
                             reader_options={"camera_model": "SIMPLE_RADIAL"}, device=pycolmap.Device.cpu)
    pycolmap.match_exhaustive(database_path=str(database), device=pycolmap.Device.cpu)
    options = pycolmap.IncrementalPipelineOptions()
    options.ba_refine_focal_length = True
    options.ba_refine_extra_params = True
    options.ba_refine_principal_point = False
    maps = pycolmap.incremental_mapping(database_path=str(database), image_path=str(sequence / "images"),
                                       output_path=str(sparse), options=options)
    if not maps:
        raise ValueError("Could not reconstruct sequence; collect richer motion/texture")
    def pair_count(model):
        names = {image.name for image in model.images.values() if image.has_pose}
        return sum(name.startswith("left/") and name.replace("left/", "right/", 1) in names for name in names)
    model = max(maps.values(), key=pair_count)
    text = workspace / "text"
    text.mkdir()
    model.write_text(str(text))
    return text


def reconstruct(executable, sequence, model):
    if model is not None:
        return model
    training, _ = split_sequence(sequence)
    executable_path = shutil.which(executable) if executable else None
    images = sequence / "images"
    workspace = sequence / ("colmap-" + uuid.uuid4().hex)
    workspace.mkdir()
    if executable_path is None:
        if executable:
            raise ValueError("Specified COLMAP executable not found: " + executable)
        return reconstruct_python(sequence, workspace, training)
    database = workspace / "database.db"
    sparse = workspace / "sparse"
    sparse.mkdir()
    image_list = workspace / "training.txt"
    image_list.write_text("\n".join(training)+"\n", encoding="utf-8")
    def run(command, *options):
        subprocess.run([executable_path, command, *map(str, options)], check=True)
    def cpu_option(command, current, legacy):
        help_result = subprocess.run([executable_path, command, "-h"], check=True, capture_output=True, text=True)
        help_text = help_result.stdout + help_result.stderr
        # COLMAP renamed these options; use only options advertised by the installed CLI.
        if current in help_text:
            return ["--" + current, "0"]
        if legacy in help_text:
            return ["--" + legacy, "0"]
        raise ValueError("Cannot identify CPU option in COLMAP " + command + " -h")
    run("feature_extractor", "--database_path", database, "--image_path", images,
        "--image_list_path", image_list,
        "--ImageReader.camera_model", "SIMPLE_RADIAL", "--ImageReader.single_camera_per_folder", "1",
        *cpu_option("feature_extractor", "FeatureExtraction.use_gpu", "SiftExtraction.use_gpu"))
    run("exhaustive_matcher", "--database_path", database,
        *cpu_option("exhaustive_matcher", "FeatureMatching.use_gpu", "SiftMatching.use_gpu"))
    run("mapper", "--database_path", database, "--image_path", images, "--output_path", sparse)
    candidates = []
    for folder in sorted(sparse.iterdir()):
        if not folder.is_dir():
            continue
        text = workspace / ("text-" + folder.name)
        text.mkdir()
        run("model_converter", "--input_path", folder, "--output_path", text, "--output_type", "TXT")
        _, reconstructed = read_model(text)
        count = sum(name.startswith("left/") and name.replace("left/", "right/", 1) in reconstructed for name in reconstructed)
        candidates.append((count, text))
    if not candidates:
        raise ValueError("COLMAP could not reconstruct this sequence; collect richer camera motion")
    return max(candidates, key=lambda item: item[0])[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence", type=Path)
    parser.add_argument("--baseline", type=float, required=True, help="Measured optical-centre separation, metres")
    parser.add_argument("--colmap", help="Optional COLMAP executable; default uses installed pycolmap")
    parser.add_argument("--model", type=Path, help="Existing COLMAP TEXT model; skip reconstruction")
    parser.add_argument("--output", type=Path, help="New raw calibration file; must not already exist")
    args = parser.parse_args()
    destination = args.output or args.sequence / "scene-calibration.properties"
    if destination.exists() or destination.with_suffix(".quality.json").exists():
        parser.error("Output already exists; use a new --output path")
    if not math.isfinite(args.baseline) or args.baseline <= 0:
        parser.error("--baseline must be a positive measured value in metres")
    try:
        model = reconstruct(args.colmap, args.sequence, args.model)
        cameras, images = read_model(model)
        result = calibrate(cameras, images, args.baseline)
        result[-1]["source_model"] = str(model.resolve())
        if args.model is None:
            _, heldout = split_sequence(args.sequence)
            validate_heldout(args.sequence, heldout, result)
        else:
            result[-1]["independent_validation"] = "Not performed: imported external model"
        write_properties(destination, result)
        print("Calibration written:", destination.resolve())
        print(json.dumps(result[-1], indent=2))
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, str(error) + "\n")


if __name__ == "__main__":
    main()
