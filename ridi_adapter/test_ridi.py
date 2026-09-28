"""Run with: python ridi_adapter/test_ridi.py"""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import ridi_features as rf  # noqa: E402
import ridi_model as rm  # noqa: E402


class AlignTests(unittest.TestCase):
    def test_flat_board_forward_is_minus_z(self):
        # A board lying flat feels gravity along +z; RIDI's stabilized frame puts gravity on +y and
        # forward (the board's +y, its nose) on -z.
        g = np.array([[0.0, 0.0, 9.8]])
        out = rf.align_with_gravity(np.array([[0.0, 1.0, 0.0], ]), g)
        np.testing.assert_allclose(out, [[0.0, 0.0, -1.0]], atol=1e-9)
        out = rf.align_with_gravity(np.array([[0.0, 0.0, 1.0]]), g)
        np.testing.assert_allclose(out, [[0.0, 1.0, 0.0]], atol=1e-9)
        out = rf.align_with_gravity(np.array([[1.0, 0.0, 0.0]]), g)  # sideways stays sideways
        np.testing.assert_allclose(out, [[1.0, 0.0, 0.0]], atol=1e-9)

    def test_already_aligned_and_upside_down(self):
        v = np.array([[0.3, 0.2, -0.5]])
        np.testing.assert_allclose(rf.align_with_gravity(v, np.array([[0.0, 9.8, 0.0]])), v)
        np.testing.assert_allclose(rf.align_with_gravity(v, np.array([[0.0, -9.8, 0.0]])), [[0.3, -0.2, 0.5]])

    def test_tilt_is_removed(self):
        # tilting the board must not change what a horizontal movement looks like after alignment
        th = np.radians(30)
        c, s = np.cos(th), np.sin(th)
        Rx = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])   # device tilted about its x axis
        g_dev = Rx.T @ np.array([0.0, 0.0, 9.8])
        push_dev = Rx.T @ np.array([0.0, 1.0, 0.0])          # a horizontal push along the room's y
        out = rf.align_with_gravity(push_dev[None], g_dev[None])[0]
        np.testing.assert_allclose(out, [0.0, 0.0, -1.0], atol=1e-9)

    def test_length_is_kept(self):
        rng = np.random.default_rng(1)
        g = rng.normal(size=(50, 3))
        v = rng.normal(size=(50, 3))
        np.testing.assert_allclose(np.linalg.norm(rf.align_with_gravity(v, g), axis=1), np.linalg.norm(v, axis=1))


class SmoothAndWorldTests(unittest.TestCase):
    def test_smoothing_keeps_a_constant(self):
        x = np.full((200, 6), 3.5)
        np.testing.assert_allclose(rf.gaussian_smooth(x), x)

    def test_feature_row_layout(self):
        n = 400
        gyro = np.tile([1.0, 2.0, 3.0], (n, 1))
        lin = np.tile([4.0, 5.0, 6.0], (n, 1))
        row = rf.features_at(gyro, lin, [300])[0]
        self.assertEqual(row.shape, (1200,))
        np.testing.assert_allclose(row[:6], [1, 2, 3, 4, 5, 6])      # frame after frame
        np.testing.assert_allclose(row[-6:], [1, 2, 3, 4, 5, 6])

    def test_forward_speed_points_along_the_nose(self):
        g = np.array([[0.0, 0.0, 9.8]])
        q = np.array([[1.0, 0.0, 0.0, 0.0]])
        v = rf.local_speed_to_world(np.array([[0.0, -1.2]]), g, q)   # 1.2 m/s forward (z is negative)
        np.testing.assert_allclose(v, [[0.0, 1.2, 0.0]], atol=1e-9)
        v = rf.local_speed_to_world(np.array([[0.5, 0.0]]), g, q)    # 0.5 m/s to the right
        np.testing.assert_allclose(v, [[0.5, 0.0, 0.0]], atol=1e-9)


@unittest.skipUnless((rm.CACHE_DIR / "regressor_0_0.npz").exists(), "model not converted yet (run try_ridi.py once)")
class ModelTests(unittest.TestCase):
    def test_a_window_of_nothing_predicts_about_zero_speed(self):
        # if the sign of the model's offset were read wrongly this would be about 2.3 m/s, not ~0
        z = rm.load_regressor(0, 1, verbose=False)
        self.assertLess(abs(z.predict(np.zeros(1200))[0]), 0.3)
        x = rm.load_regressor(0, 0, verbose=False)
        self.assertLess(abs(x.predict(np.zeros(1200))[0]), 0.3)


if __name__ == "__main__":
    unittest.main()
