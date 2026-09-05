from types import SimpleNamespace

import pytest

from tools.check_onboard_clock import decode_reply, timestamp, within_limit
from app.observation_pipeline import ObservationContract, ObservationPipeline


def make_reply(offset=0.203):
    sent = 1788480000.0
    stamp = timestamp(sent)
    reply = bytearray(48)
    reply[0] = 0x24
    reply[1] = 2
    reply[24:32] = stamp
    reply[32:40] = timestamp(sent + offset + 0.005)
    reply[40:48] = timestamp(sent + offset + 0.006)
    return reply, stamp, sent, sent + 0.011


def test_valid_offset():
    offset, delay, stratum = decode_reply(*make_reply())
    assert offset == pytest.approx(203, abs=0.01)
    assert delay == pytest.approx(10, abs=0.01)
    assert stratum == 2


@pytest.mark.parametrize("offset,accepted", [(300, True), (-300, True), (300.1, False), (-300.1, False), (float("nan"), False)])
def test_absolute_limit(offset, accepted):
    assert within_limit(offset, 300) is accepted


@pytest.mark.parametrize("kind", ["kod", "unsync", "mode", "origin", "short", "empty"])
def test_invalid_packet_fails_closed(kind):
    reply, stamp, sent, received = make_reply()
    if kind == "kod":
        reply[1] = 0
        reply[12:16] = b"RATE"
        reply[32:40] = reply[40:48] = stamp  # would look near zero without header validation
    elif kind == "unsync":
        reply[0] |= 0xC0
    elif kind == "mode":
        reply[0] = 0x23
    elif kind == "origin":
        reply[24:32] = bytes(8)
    elif kind == "short":
        reply = reply[:47]
    else:
        reply[40:48] = bytes(8)
    with pytest.raises(ValueError):
        decode_reply(reply, stamp, sent, received)


def test_observation_future_limit_keeps_sensor_sync_limit(monkeypatch):
    monkeypatch.setattr("app.observation_pipeline.time.time", lambda: 1000)
    contract = ObservationContract(5, 750, 80, "world", "base_link", "usb", "cal")
    assert contract.max_future_skew_ms == 300
    pipeline = ObservationPipeline(contract, None, None, None, 500)
    with pytest.raises(ValueError, match="clock-unsynchronized"):
        pipeline._validate_observation(SimpleNamespace(capture_unix_ms=1000301))
    # Exactly 300 ms future passes clock check, but 81 ms image/odom skew still fails.
    observation = SimpleNamespace(capture_unix_ms=1000300, odometry=SimpleNamespace(stamp_unix_ms=1000219))
    with pytest.raises(ValueError, match="not time-synchronized"):
        pipeline._validate_observation(observation)
