"""Tests use synthetic multi-camera poses; no installed COLMAP required."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import sys
import numpy as np

sys.dont_write_bytecode = True

spec = importlib.util.spec_from_file_location("self_calibrate", Path(__file__).with_name("self-calibrate.py"))
calibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibration)


def scene():
    cameras = {1: ("SIMPLE_RADIAL", 640, 480, np.array([500, 320, 240, .01])),
               2: ("SIMPLE_RADIAL", 640, 480, np.array([505, 321, 239, .02]))}
    images = {}
    rig_rotation = calibration.rotation([np.cos(.005), 0, np.sin(.005), 0])
    translation = np.array([-.2, .002, .001])  # Unknown SfM units.
    for i in range(20):
        q = [np.cos(i*.01), 0, np.sin(i*.01), 0]
        r = calibration.rotation(q)
        center = np.array([i*.15, .3*np.sin(i*.5), .05*np.cos(i*.3)])
        t = -r @ center
        images[f"left/{i:06d}.png"] = (1, r, t)
        images[f"right/{i:06d}.png"] = (2, rig_rotation @ r, rig_rotation @ t+translation)
    return cameras, images, rig_rotation, translation


class CalibrationTest(unittest.TestCase):
    def test_relative_pose_and_metric_baseline(self):
        cameras, images, expected_r, expected_t = scene()
        left, right, r, t, report = calibration.calibrate(cameras, images, .061)
        np.testing.assert_allclose(r, expected_r, atol=1e-12)
        np.testing.assert_allclose(t, expected_t/np.linalg.norm(expected_t)*.061, atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(t), .061)
        self.assertEqual(report["pairs"], 20)
        self.assertEqual(left[3], [.01, 0, 0, 0, 0])
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "calibration.properties"
            calibration.write_properties(output, (left, right, r, t, report))
            self.assertIn("rectified=false", output.read_text())
            self.assertTrue(output.with_suffix(".quality.json").exists())
            with self.assertRaises(FileExistsError):
                calibration.write_properties(output, (left, right, r, t, report))

    def test_rejects_inconsistent_rig(self):
        cameras, images, _, _ = scene()
        for i in range(8):
            key = f"right/{i:06d}.png"
            cid, r, t = images[key]
            images[key] = (cid, r, t+np.array([0, .15, 0]))
        with self.assertRaisesRegex(ValueError, "inconsistent"):
            calibration.calibrate(cameras, images, .061)

    def test_rejects_pure_rotation_and_unknown_scale(self):
        cameras, images, _, _ = scene()
        for i in range(20):
            for eye in ["left", "right"]:
                key = f"{eye}/{i:06d}.png"
                cid, r, _ = images[key]
                images[key] = (cid, r, np.array([0., 0., 0.]) if eye == "left" else np.array([-.2, 0., 0.]))
        with self.assertRaisesRegex(ValueError, "excitation"):
            calibration.calibrate(cameras, images, .061)
        with self.assertRaisesRegex(ValueError, "baseline"):
            calibration.calibrate(cameras, images, float("nan"))

    def test_reads_pose_lines_without_skipping_empty_observations(self):
        with tempfile.TemporaryDirectory() as folder:
            model = Path(folder)
            (model / "cameras.txt").write_text("# cameras\n1 SIMPLE_RADIAL 640 480 500 320 240 0\n")
            (model / "images.txt").write_text("# images\n1 1 0 0 0 0 0 0 1 left/000001.png\n\n2 1 0 0 0 -.2 0 0 1 right/000001.png\n10 20 -1\n")
            cameras, images = calibration.read_model(model)
            self.assertEqual(len(images), 2)
            np.testing.assert_allclose(images["right/000001.png"][2], [-.2, 0, 0])


if __name__ == "__main__":
    unittest.main()
