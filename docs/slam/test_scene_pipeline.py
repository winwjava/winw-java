"""Optional real SfM integration test. Generates synthetic textured 3D surfaces, not camera footage.

Run with the self-calibration venv. Artifacts are saved under target/scene-pipeline-test-*.
"""
import importlib.util
from pathlib import Path
import uuid
import sys
import cv2
import numpy as np

sys.dont_write_bytecode = True

spec = importlib.util.spec_from_file_location("scene_calibration", Path(__file__).with_name("self-calibrate.py"))
calibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibration)


def generate(root):
    random = np.random.default_rng(64123)
    k = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
    patches = []
    for y in range(5):
        for x in range(7):
            z = 1.7+random.random()*3
            center = np.array([(50+x*85-320)*z/500, (40+y*88-240)*z/500, z])
            half = 32*z/500
            corners = center+np.array([[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]])
            texture = random.integers(20, 235, (100, 100), dtype=np.uint8)
            texture = cv2.GaussianBlur(texture, (3, 3), .6)
            for _ in range(35):
                px, py = random.integers(4, 96, 2)
                cv2.circle(texture, (int(px), int(py)), int(random.integers(1, 6)), int(random.integers(0, 255)), -1)
            patches.append((z, corners, texture))
    patches.sort(reverse=True, key=lambda item: item[0])
    for eye in ["left", "right"]:
        (root / "images" / eye).mkdir(parents=True)
    (root / "manifest.txt").write_text("# SYNTHETIC textured scene, not camera footage\n"+
        "".join(f"{i*.1:.1f} images/left/{i:06d}.png images/right/{i:06d}.png\n" for i in range(30)), encoding="utf-8")
    rig_r = cv2.Rodrigues(np.array([0., .008, .007]))[0]
    rig_t = np.array([-.061, .0004, .0002])
    for i in range(30):
        phase = i/29
        center = np.array([-.5+phase, .09*np.sin(phase*2*np.pi), .025*np.cos(phase*2*np.pi)])
        rl = cv2.Rodrigues(np.array([.025*np.sin(phase*2*np.pi), -.08+.16*phase, .01*np.cos(phase*2*np.pi)]))[0]
        tl = -rl @ center
        for eye, r, t in [("left", rl, tl), ("right", rig_r @ rl, rig_r @ tl+rig_t)]:
            image = np.full((480, 640), 30, np.uint8)
            for _, corners, texture in patches:
                points = corners @ r.T+t
                projected = points @ k.T
                uv = (projected[:, :2]/projected[:, 2:]).astype(np.float32)
                h = cv2.getPerspectiveTransform(np.array([[0, 0], [99, 0], [99, 99], [0, 99]], np.float32), uv)
                warped = cv2.warpPerspective(texture, h, (640, 480))
                mask = cv2.warpPerspective(np.full((100, 100), 255, np.uint8), h, (640, 480))
                image[mask > 250] = warped[mask > 250]
            assert cv2.imwrite(str(root / "images" / eye / f"{i:06d}.png"), image)
    return rig_r, rig_t


def main():
    root = Path(__file__).resolve().parents[2] / "target" / ("scene-pipeline-test-"+uuid.uuid4().hex)
    root.mkdir()
    expected_r, expected_t = generate(root)
    model = calibration.reconstruct(None, root, None)
    result = calibration.calibrate(*calibration.read_model(model), np.linalg.norm(expected_t))
    calibration.validate_heldout(root, calibration.split_sequence(root)[1], result)
    calibration.write_properties(root / "scene-calibration.properties", result)
    left, right, r, t, report = result
    # Independent known synthetic geometry checks metric scale and recovered focal length.
    assert abs(left[2][0]-500)/500 < .08, left[2]
    assert abs(right[2][0]-500)/500 < .08, right[2]
    assert calibration.angle(r, expected_r) < .5
    assert np.linalg.norm(t-expected_t) < .003
    print("SYNTHETIC integration passed:", root)
    print(report)


if __name__ == "__main__":
    main()
