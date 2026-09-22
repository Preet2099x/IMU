"""Run with: python visualizer/test_bridge.py"""
import json
import tempfile
import unittest
from pathlib import Path

from bridge import (DEMO_MOVE, DEMO_PATH, DEMO_REST, G, CalStore, Hub, cal_command, demo_raw, demo_sample,
                    handle_line, parse_acal, parse_viz, valid_cal)


class ParseTests(unittest.TestCase):
    def test_real_firmware_line(self):
        ev = parse_viz("VIZ,66106,0.9969,0.0391,0.0681,-0.0022,-0.4258,0.4343,0.7746,0.02,-0.15,0.26")
        self.assertEqual(ev["ms"], 66106.0)
        self.assertEqual(ev["q"], [0.9969, 0.0391, 0.0681, -0.0022])
        self.assertEqual(ev["f"], [-0.4258, 0.4343, 0.7746])
        self.assertEqual(ev["g"], [0.02, -0.15, 0.26])

    def test_bad_lines(self):
        self.assertIsNone(parse_viz("Heading:  0.2  Pitch:  -6.3  Roll:-135.9"))
        self.assertIsNone(parse_viz("VIZ,1,2,3"))
        self.assertIsNone(parse_viz("VIZ,1,2,3,4,5,6,7,8,9,10,nan_x"))
        self.assertIsNone(parse_viz("VIZ,66106,0.99,0.03"))  # a line cut short by the port

    def test_board_confirms_calibration(self):
        self.assertEqual(parse_acal("ACAL,0.05000,0.00000,-0.01000,1.00400,0.99500,0.97900"),
                         {"offset": [0.05, 0.0, -0.01], "scale": [1.004, 0.995, 0.979]})
        self.assertIsNone(parse_acal("ACAL,1,2"))


class HubTests(unittest.TestCase):
    def test_lines_are_routed(self):
        hub = Hub()
        q = hub.subscribe()
        q.get_nowait()  # the status every new client gets first
        self.assertEqual(handle_line(hub, "VIZ,1,1,0,0,0,0,0,1,0,0,0"), "viz")
        self.assertEqual(handle_line(hub, "Heading: 10.0 Pitch: 1.0 Roll: 2.0"), "text")
        self.assertIsNone(handle_line(hub, "DIAG rest=1"))
        self.assertIsNone(handle_line(hub, "ACAL,0.01,0,0,1,1,1"))
        self.assertIsNone(handle_line(hub, "=== running ==="))
        self.assertEqual(q.get_nowait()["type"], "viz")
        self.assertEqual(q.get_nowait(), {"type": "acal", "offset": [0.01, 0.0, 0.0], "scale": [1.0, 1.0, 1.0]})
        self.assertEqual(q.get_nowait(), {"type": "log", "text": "=== running ==="})
        self.assertTrue(q.empty())  # the text line and DIAG line were not forwarded

    def test_status_is_replayed_to_new_clients(self):
        hub = Hub()
        hub.set_status("connected", "COM7", "Connected to COM7", session=3)
        status = hub.subscribe().get_nowait()
        self.assertEqual((status["state"], status["port"], status["session"]), ("connected", "COM7", 3))


class CalibrationTests(unittest.TestCase):
    good = {"offset": [0.006, -0.010, 0.014], "scale": [1.004, 0.995, 0.979]}

    def test_validation(self):
        self.assertTrue(valid_cal(self.good))
        for bad in ({"offset": [0, 0], "scale": [1, 1, 1]}, {"offset": [0.9, 0, 0], "scale": [1, 1, 1]},
                    {"offset": [0, 0, 0], "scale": [3, 1, 1]}, {"offset": [0, 0, 0], "scale": [1, 1, float("nan")]},
                    {"offset": "abc", "scale": [1, 1, 1]}, {}):
            self.assertFalse(valid_cal(bad), bad)

    def test_firmware_command(self):
        self.assertEqual(cal_command(self.good), b"c,0.006000,-0.010000,0.014000,1.004000,0.995000,0.979000\n")

    def test_stored_in_a_file_and_read_back(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "cal.json"
            self.assertEqual(CalStore(path).get()["scale"], [1.0, 1.0, 1.0])  # nothing saved yet
            CalStore(path).set(self.good)
            self.assertEqual(json.loads(path.read_text())["offset"], self.good["offset"])
            self.assertEqual(CalStore(path).get(), self.good)
            path.write_text("not json")
            self.assertEqual(CalStore(path).get()["scale"], [1.0, 1.0, 1.0])  # a damaged file is ignored
            with self.assertRaises(ValueError):
                CalStore(path).set({"offset": [5, 0, 0], "scale": [1, 1, 1]})

    def test_demo_mode_never_touches_a_file(self):
        store = CalStore(None)
        store.set(self.good)
        self.assertEqual(store.get(), self.good)


class DemoRouteTests(unittest.TestCase):
    def test_route_reaches_every_waypoint_when_integrated(self):
        dt, pos, vel, t = 0.002, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0
        period = DEMO_REST + DEMO_MOVE
        for leg, wp in enumerate(DEMO_PATH):
            end = (leg + 1) * period
            while t < end:
                acc, _ = demo_sample(t)
                for i in range(3):
                    vel[i] += acc[i] * dt
                    pos[i] += vel[i] * dt
                t += dt
            for i in range(3):
                self.assertAlmostEqual(pos[i], wp[i], delta=0.01, msg=f"leg {leg} axis {i}")
            self.assertLess(max(abs(v) for v in vel), 0.01)

    def test_raw_accelerometer_is_gravity_plus_the_push(self):
        # A perfect sensor at rest, flat, reads 1 g up.
        f = demo_raw((0, 0, 0), (0, 0), scale=(1, 1, 1), offset=(0, 0, 0))
        self.assertAlmostEqual(f[2], 1.0, places=9)
        self.assertAlmostEqual(abs(f[0]) + abs(f[1]), 0.0, places=9)
        # Pushed forward 2 m/s^2 while level: the sensor sees +x specific force.
        f = demo_raw((2.0, 0, 0), (0, 0), scale=(1, 1, 1), offset=(0, 0, 0))
        self.assertAlmostEqual(f[0], 2.0 / G, places=9)
        # Tilted 30 degrees, the length is still 1 g.
        f = demo_raw((0, 0, 0), (20.0, 30.0), scale=(1, 1, 1), offset=(0, 0, 0))
        self.assertAlmostEqual(sum(v * v for v in f), 1.0, places=9)


if __name__ == "__main__":
    unittest.main()
