#!/usr/bin/env python3
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from command_protocol import ProtocolError, validate_command


LIMITS = {
    "command_ttl_sec": 10.0,
    "max_clock_skew_sec": 5.0,
    "max_move_distance": 3.0,
    "max_rotate_angle_deg": 180.0,
    "max_orbit_radius": 5.0,
    "max_orbit_angle_deg": 360.0,
}


def command(action="HOLD", arguments=None, confirmed=True, now=1_000_000):
    return {
        "type": "command", "protocol_version": "1.0",
        "request_id": "12345678-test", "client_id": "android-test",
        "issued_at_ms": now, "expires_at_ms": now + 9000,
        "action": action, "arguments": arguments or {},
        "operator_confirmed": confirmed,
    }


class ProtocolTest(unittest.TestCase):
    def test_hold_is_valid_without_confirmation(self):
        result = validate_command(command(confirmed=False), LIMITS, now_ms=1_000_100)
        self.assertEqual("HOLD", result["action"])

    def test_move_normalizes_direction(self):
        result = validate_command(command("MOVE", {"direction": "forward", "distance_m": 1.2}),
                                  LIMITS, now_ms=1_000_100)
        self.assertEqual("FORWARD", result["arguments"]["direction"])
        self.assertEqual(1.2, result["arguments"]["distance_m"])

    def test_missing_move_distance_is_allowed_for_onboard_default(self):
        result = validate_command(command("MOVE", {"direction": "UP"}),
                                  LIMITS, now_ms=1_000_100)
        self.assertNotIn("distance_m", result["arguments"])

    def test_rejects_excessive_distance(self):
        with self.assertRaises(ProtocolError) as caught:
            validate_command(command("MOVE", {"direction": "FORWARD", "distance_m": 99}),
                             LIMITS, now_ms=1_000_100)
        self.assertEqual("OUT_OF_RANGE", caught.exception.code)

    def test_rejects_unknown_action(self):
        with self.assertRaises(ProtocolError) as caught:
            validate_command(command("RAW_ATTITUDE", {}), LIMITS, now_ms=1_000_100)
        self.assertEqual("ACTION_NOT_ALLOWED", caught.exception.code)

    def test_takeoff_profile_is_fixed(self):
        result = validate_command(command("START_TAKEOFF_LOCALIZE_ORBIT", {
            "mission_profile": "takeoff_lidar_localization_orbit"
        }), LIMITS, now_ms=1_000_100)
        self.assertEqual("START_TAKEOFF_LOCALIZE_ORBIT", result["action"])

    def test_rejects_unconfirmed_motion(self):
        with self.assertRaises(ProtocolError) as caught:
            validate_command(command("ROTATE", {"direction": "CLOCKWISE"}, confirmed=False),
                             LIMITS, now_ms=1_000_100)
        self.assertEqual("CONFIRMATION_REQUIRED", caught.exception.code)

    def test_rejects_expired_command(self):
        payload = command()
        payload["expires_at_ms"] = 999_999
        with self.assertRaises(ProtocolError) as caught:
            validate_command(payload, LIMITS, now_ms=1_000_100)
        self.assertEqual("EXPIRED", caught.exception.code)


if __name__ == "__main__":
    unittest.main()
