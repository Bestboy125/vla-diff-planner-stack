#!/usr/bin/env python3
"""Pure protocol validation. This module intentionally has no ROS dependency."""

import math
import time


ALLOWED_ACTIONS = {
    "START_TAKEOFF_LOCALIZE_ORBIT",
    "MOVE",
    "ROTATE",
    "ORBIT",
    "HOLD",
    "LAND",
    "EMERGENCY_STOP",
}
MOVE_DIRECTIONS = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "UP", "DOWN"}
ROTATE_DIRECTIONS = {"CLOCKWISE", "COUNTERCLOCKWISE"}
ORBIT_DIRECTIONS = {"CLOCKWISE", "COUNTERCLOCKWISE"}


class ProtocolError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code
        self.message = message


def _number(arguments, name, minimum, maximum, required=False):
    value = arguments.get(name)
    if value is None:
        if required:
            raise ProtocolError("INVALID_ARGUMENT", "%s is required" % name)
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ProtocolError("INVALID_ARGUMENT", "%s must be a finite number" % name)
    value = float(value)
    if value < minimum or value > maximum:
        raise ProtocolError("OUT_OF_RANGE", "%s must be in [%s, %s]" % (name, minimum, maximum))
    return value


def validate_command(payload, limits, now_ms=None):
    if not isinstance(payload, dict) or payload.get("type") != "command":
        raise ProtocolError("INVALID_MESSAGE", "type must be command")
    if payload.get("protocol_version") != "1.0":
        raise ProtocolError("UNSUPPORTED_VERSION", "protocol_version must be 1.0")
    request_id = payload.get("request_id")
    if not isinstance(request_id, str) or not 8 <= len(request_id) <= 80:
        raise ProtocolError("INVALID_REQUEST_ID", "request_id length must be 8..80")
    action = payload.get("action")
    if action not in ALLOWED_ACTIONS:
        raise ProtocolError("ACTION_NOT_ALLOWED", "action is not allow-listed")
    if action != "HOLD" and payload.get("operator_confirmed") is not True:
        raise ProtocolError("CONFIRMATION_REQUIRED", "operator confirmation is required")

    current = int(time.time() * 1000) if now_ms is None else int(now_ms)
    issued = payload.get("issued_at_ms")
    expires = payload.get("expires_at_ms")
    if not isinstance(issued, int) or not isinstance(expires, int):
        raise ProtocolError("INVALID_TIMESTAMP", "integer timestamps are required")
    skew_ms = int(limits["max_clock_skew_sec"] * 1000)
    ttl_ms = int(limits["command_ttl_sec"] * 1000)
    if issued > current + skew_ms:
        raise ProtocolError("CLOCK_SKEW", "issued_at_ms is in the future")
    if expires < current or expires - issued > ttl_ms:
        raise ProtocolError("EXPIRED", "command expired or lifetime is too long")

    arguments = payload.get("arguments") or {}
    if not isinstance(arguments, dict):
        raise ProtocolError("INVALID_ARGUMENT", "arguments must be an object")
    normalized = dict(arguments)

    if action == "MOVE":
        direction = str(arguments.get("direction", "")).upper()
        if direction not in MOVE_DIRECTIONS:
            raise ProtocolError("INVALID_ARGUMENT", "invalid MOVE direction")
        normalized["direction"] = direction
        distance = _number(arguments, "distance_m", 0.05, limits["max_move_distance"])
        if distance is not None:
            normalized["distance_m"] = distance
    elif action == "ROTATE":
        direction = str(arguments.get("direction", "")).upper()
        if direction not in ROTATE_DIRECTIONS:
            raise ProtocolError("INVALID_ARGUMENT", "invalid ROTATE direction")
        normalized["direction"] = direction
        angle = _number(arguments, "angle_deg", 1.0, limits["max_rotate_angle_deg"])
        if angle is not None:
            normalized["angle_deg"] = angle
    elif action == "ORBIT":
        direction = arguments.get("direction")
        if direction is not None:
            direction = str(direction).upper()
            if direction not in ORBIT_DIRECTIONS:
                raise ProtocolError("INVALID_ARGUMENT", "invalid ORBIT direction")
            normalized["direction"] = direction
        radius = _number(arguments, "radius_m", 0.2, limits["max_orbit_radius"])
        angle = _number(arguments, "orbit_angle_deg", 5.0, limits["max_orbit_angle_deg"])
        if radius is not None:
            normalized["radius_m"] = radius
        if angle is not None:
            normalized["orbit_angle_deg"] = angle
    elif action == "START_TAKEOFF_LOCALIZE_ORBIT":
        if arguments.get("mission_profile") != "takeoff_lidar_localization_orbit":
            raise ProtocolError("INVALID_ARGUMENT", "unsupported mission_profile")

    return {
        "request_id": request_id,
        "client_id": str(payload.get("client_id", "")),
        "action": action,
        "arguments": normalized,
        "source_text": str(payload.get("source_text", ""))[:500],
        "issued_at_ms": issued,
        "expires_at_ms": expires,
    }
