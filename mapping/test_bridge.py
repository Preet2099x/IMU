"""Run with: python mapping/test_bridge.py"""
import unittest

from bridge import Hub, Simulator, handle_line, parse_attitude


class ParseTests(unittest.TestCase):
    def test_real_firmware_line(self):
        ev = parse_attitude("Heading:  0.2  Pitch:  -6.3  Roll:-135.9  Gyro(dps) X:   0.00 Y:   0.07 Z:  -0.12")
        self.assertEqual((ev["heading"], ev["pitch"], ev["roll"]), (0.2, -6.3, -135.9))
        self.assertEqual(ev["gyro"], [0.0, 0.07, -0.12])

    def test_line_without_gyro(self):
        ev = parse_attitude("Heading: 297.0  Pitch:  -6.1  Roll:  43.2")
        self.assertEqual((ev["heading"], ev["pitch"], ev["roll"]), (297.0, -6.1, 43.2))
        self.assertNotIn("gyro", ev)

    def test_non_attitude_lines(self):
        self.assertIsNone(parse_attitude("gyro bias (dps): x=0.154 y=0.198 z=-0.028"))
        self.assertIsNone(parse_attitude("DIAG rest=0 rate=0.389 |a|=0.9700 bias=0.1357 0.1914 -0.0244"))
        self.assertIsNone(parse_attitude("Heading: nan Pitch: nan Roll: nan"))


class HubTests(unittest.TestCase):
    def test_lines_are_routed(self):
        hub = Hub()
        q = hub.subscribe()
        self.assertEqual(q.get_nowait()["type"], "status")
        handle_line(hub, "Heading: 10.0 Pitch: 1.0 Roll: 2.0")
        handle_line(hub, "DIAG rest=1 odr=400.0")
        handle_line(hub, "=== running ===")
        handle_line(hub, "   ")
        self.assertEqual(q.get_nowait()["type"], "att")
        self.assertEqual(q.get_nowait(), {"type": "log", "text": "=== running ==="})
        self.assertTrue(q.empty())

    def test_status_carries_session_and_is_replayed_to_new_clients(self):
        hub = Hub()
        hub.set_status("connected", "COM7", "Connected to COM7", session=3)
        late = hub.subscribe()  # a browser tab that opens after the board connected
        status = late.get_nowait()
        self.assertEqual((status["state"], status["port"], status["session"]), ("connected", "COM7", 3))

    def test_slow_client_drops_oldest(self):
        hub = Hub()
        q = hub.subscribe()
        for i in range(600):
            hub.publish({"type": "log", "text": str(i)})
        self.assertEqual(q.qsize(), 500)
        texts = [q.get_nowait().get("text") for _ in range(500)]
        self.assertEqual(texts[-1], "599")


class SimulatorTests(unittest.TestCase):
    def test_pose_is_continuous_and_parseable(self):
        prev = Simulator.pose(0.0)
        t = 0.0
        while t < 33.0:
            t += 0.1
            cur = Simulator.pose(t)
            for c, p in zip(cur, prev):
                self.assertLess(abs((c - p + 540) % 360 - 180), 50.0, f"jump at t={t:.1f}")
            prev = cur
            roll, pitch, yaw = cur
            self.assertIsNotNone(parse_attitude(f"Heading:{yaw:6.1f}  Pitch:{pitch:6.1f}  Roll:{roll:6.1f}"))


if __name__ == "__main__":
    unittest.main()
