"""Read-only NTP preflight. Reject KoD replies instead of treating them as time."""

import argparse
import math
import socket
import struct
import time

NTP_EPOCH = 2208988800


def timestamp(unix_seconds):
    value = unix_seconds + NTP_EPOCH
    seconds = int(value)
    return struct.pack("!II", seconds, int((value - seconds) * 2**32))


def read_timestamp(packet, offset):
    seconds, fraction = struct.unpack_from("!II", packet, offset)
    return seconds - NTP_EPOCH + fraction / 2**32


def decode_reply(packet, request_stamp, sent_at, received_at):
    if len(packet) < 48:
        raise ValueError("short NTP reply")
    leap, version, mode = packet[0] >> 6, (packet[0] >> 3) & 7, packet[0] & 7
    if packet[1] == 0:
        raise ValueError("NTP KoD/refusal: " + packet[12:16].decode("ascii", errors="replace"))
    if leap == 3 or not 1 <= packet[1] <= 15 or mode != 4 or version not in (3, 4):
        raise ValueError("NTP server is unsynchronized or reply header is invalid")
    if packet[24:32] != request_stamp:
        raise ValueError("NTP reply does not match this request")
    if packet[32:40] == bytes(8) or packet[40:48] == bytes(8):
        raise ValueError("NTP reply has missing timestamps")
    server_receive = read_timestamp(packet, 32)
    server_send = read_timestamp(packet, 40)
    if server_send < server_receive:
        raise ValueError("server clock moved backwards during measurement")
    offset_ms = ((server_receive - sent_at) + (server_send - received_at)) * 500
    delay_ms = ((received_at - sent_at) - (server_send - server_receive)) * 1000
    if delay_ms < -1:
        raise ValueError("invalid negative NTP round-trip delay")
    return offset_ms, max(0.0, delay_ms), packet[1]


def probe(host):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.settimeout(3)
        client.connect((host, 123))
        request = bytearray(48)
        request[0] = 0x23  # NTP v4 client
        request[2] = 4  # Advertise 16-second poll, not a burst.
        sent_at = time.time()
        started = time.monotonic()
        request[40:48] = timestamp(sent_at)
        client.send(request)
        reply = client.recv(512)
        received_at = time.time()
        elapsed = time.monotonic() - started
        # Older Windows Python builds can quantize wall/monotonic clocks at
        # roughly 15.6 ms. Allow that resolution, but reject a material step.
        if abs((received_at - sent_at) - elapsed) > 0.05:
            raise ValueError("host clock changed during measurement; retry before flight")
        return decode_reply(reply, request[40:48], sent_at, received_at)


def within_limit(offset_ms, limit_ms):
    return math.isfinite(offset_ms) and abs(offset_ms) <= limit_ms


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--max-offset-ms", type=float, default=300)
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--interval", type=float, default=10)
    args = parser.parse_args()
    if not math.isfinite(args.max_offset_ms) or args.max_offset_ms <= 0:
        parser.error("max-offset-ms must be finite and positive")
    if args.samples < 1 or not math.isfinite(args.interval) or args.interval < 10:
        parser.error("samples must be positive; interval must be at least 10 seconds (NTP rate limits)")
    try:
        for index in range(args.samples):
            if index:
                time.sleep(args.interval)
            offset, delay, stratum = probe(args.host)
            print(f"sample={index + 1} offset_ms={offset:+.3f} rtt_ms={delay:.3f} stratum={stratum}", flush=True)
            if not within_limit(offset, args.max_offset_ms):
                raise ValueError(f"absolute clock offset exceeds {args.max_offset_ms:g} ms")
    except (OSError, ValueError) as exc:
        print(f"FAIL: {exc}", flush=True)
        return 1
    print(f"PASS: all {args.samples} samples within +/-{args.max_offset_ms:g} ms; clock check only, not flight clearance.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
