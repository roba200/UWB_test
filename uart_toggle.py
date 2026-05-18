"""
uart_toggle.py
Toggles the UWB firmware between TAG and STOP modes via UART.
Implements distance-threshold handover: when the tag reports a distance
greater than --threshold for --consec consecutive readings, the current
TAG session is stopped immediately and restarted so the tag can
re-discover a nearer anchor.

Usage:
    python uart_toggle.py --port COM35
    python uart_toggle.py --port COM35 --threshold 300 --consec 3
    python uart_toggle.py --port COM35 --tag-duration 10 --stop-duration 0.2
    python uart_toggle.py --port COM35 --start-with stop

Firmware output parsed:
    JSxxxx{"TAG":{"a16":"XXXX","N":"XXXX","R":d,"Xcm":d,"Ycm":d,"O":d}}
    distance = sqrt(Xcm^2 + Ycm^2)   (Xcm/Ycm = -8531 means "not yet computed")
"""

import argparse
import json
import math
import re
import sys
import time
from typing import Optional

import serial


# ── defaults ──────────────────────────────────────────────────────────────────
DEFAULT_PORT              = "COM35"
DEFAULT_BAUD              = 115200
DEFAULT_TAG_DURATION      = 1.0    # seconds: max time TAG mode stays active
DEFAULT_STOP_DURATION     = 0.05   # seconds: STOP (idle) dwell time
DEFAULT_DIST_THRESHOLD_CM = 300.0  # cm: euclidean distance from anchor
DEFAULT_CONSEC_COUNT      = 3      # consecutive over-threshold readings before handover

# int16_t(0xDEAD) = -8531 — firmware sentinel meaning "coordinates not yet computed"
_SENTINEL = -8531

# Matches the "JSxxxx" length-prefix the firmware prepends to every JSON line
_JS_PREFIX = re.compile(r"^JS[0-9A-Fa-f]{4}")

TICK_IDLE = 0.001  # sleep only when RX buffer is empty (seconds)


# ── helpers ───────────────────────────────────────────────────────────────────

def send_command(ser: serial.Serial, cmd: str) -> None:
    """Send a plain-text command terminated with \\n (as cmd.c expects)."""
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()


def drain(ser: serial.Serial) -> list:
    """Return all complete lines currently waiting in the RX buffer."""
    if not ser.in_waiting:
        return []
    raw = ser.read(ser.in_waiting).decode("ascii", errors="replace")
    return [l for l in raw.splitlines() if l.strip()]


def parse_tag_msg(line: str) -> Optional[dict]:
    """
    Strip the JS-prefix and parse a TAG JSON message.
    Returns the inner TAG dict, or None if the line is not a TAG message.

    Example input:
        JS001E{"TAG":{"a16":"AABB","N":"CD12","R":7,"Xcm":120,"Ycm":85,"O":14}}
    """
    line = line.strip()
    m = _JS_PREFIX.match(line)
    if m:
        line = line[m.end():]
    try:
        obj = json.loads(line)
        return obj.get("TAG")
    except (json.JSONDecodeError, ValueError):
        return None


def euclidean_cm(xcm: int, ycm: int) -> Optional[float]:
    """
    Compute euclidean distance from anchor.
    Returns None when either axis carries the sentinel (not yet computed).
    """
    if xcm == _SENTINEL or ycm == _SENTINEL:
        return None
    return math.sqrt(xcm ** 2 + ycm ** 2)


# ── phase runners ─────────────────────────────────────────────────────────────

def run_stop_phase(ser: serial.Serial, duration: float) -> None:
    """Hold STOP state for `duration` seconds, printing any firmware output."""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        for line in drain(ser):
            print(f"  << {line}")
        if not ser.in_waiting:
            time.sleep(TICK_IDLE)


def run_tag_phase(ser: serial.Serial, max_duration: float,
                  threshold_cm: float, consec_required: int) -> str:
    """
    Run TAG mode until either:
      - max_duration expires   → returns 'timeout'
      - distance threshold exceeded for consec_required consecutive
        readings                → returns 'handover'

    Prints every TAG location line as it arrives.
    """
    consec = 0
    deadline = time.monotonic() + max_duration

    while time.monotonic() < deadline:
        for raw in drain(ser):
            tag = parse_tag_msg(raw)

            if tag is not None:
                xcm  = tag.get("Xcm", _SENTINEL)
                ycm  = tag.get("Ycm", _SENTINEL)
                dist = euclidean_cm(xcm, ycm)

                if dist is not None:
                    over = dist > threshold_cm
                    flag = "  <<< OVER THRESHOLD" if over else ""
                    print(f"  a16={tag.get('a16','?'):4s}  N={tag.get('N','?'):4s}"
                          f"  dist={dist:7.1f} cm  R={tag.get('R','?'):4}{flag}")
                    if over:
                        consec += 1
                        if consec >= consec_required:
                            print(f"\n  [HANDOVER] dist={dist:.1f} cm exceeded "
                                  f"{threshold_cm:.0f} cm threshold for "
                                  f"{consec_required} consecutive readings")
                            return "handover"
                    else:
                        consec = 0
                else:
                    # Sentinel — coordinates not computed yet, reset counter
                    consec = 0
                    print(f"  a16={tag.get('a16','?'):4s}  N={tag.get('N','?'):4s}"
                          f"  dist=  pending   R={tag.get('R','?'):4}")
            else:
                # Non-TAG firmware output (ok/error/diag lines)
                if raw.strip():
                    print(f"  << {raw.strip()}")

        if not ser.in_waiting:
            time.sleep(TICK_IDLE)

    return "timeout"


# ── main loop ─────────────────────────────────────────────────────────────────

def run(port: str, baud: int, tag_duration: float, stop_duration: float,
        threshold_cm: float, consec_count: int, start_with: str) -> None:

    print(f"Opening {port} at {baud} baud ...")
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,          # non-blocking reads — we poll with drain()
        )
    except serial.SerialException as exc:
        sys.exit(f"Cannot open port: {exc}")

    print(f"  TAG max duration   : {tag_duration} s")
    print(f"  STOP duration      : {stop_duration} s")
    print(f"  Distance threshold : {threshold_cm} cm  (euclidean from anchor)")
    print(f"  Consecutive count  : {consec_count}")
    print(f"  Starting with      : {start_with.upper()}")
    print("Press Ctrl-C to quit.\n")

    try:
        # Optional initial STOP to ensure a clean firmware state
        if start_with.lower() == "stop":
            print(f"[{time.strftime('%H:%M:%S')}] Sending: STOP  (initial)")
            send_command(ser, "STOP")
            run_stop_phase(ser, stop_duration)

        while True:
            # ── TAG phase ────────────────────────────────────────────────────
            print(f"[{time.strftime('%H:%M:%S')}] Sending: TAG")
            send_command(ser, "TAG")

            result = run_tag_phase(ser, tag_duration, threshold_cm, consec_count)

            # ── STOP phase ───────────────────────────────────────────────────
            reason = "handover" if result == "handover" else "normal timeout"
            print(f"\n[{time.strftime('%H:%M:%S')}] Sending: STOP  ({reason})")
            send_command(ser, "STOP")
            run_stop_phase(ser, stop_duration)
            # Loop back → new TAG → tag re-discovers nearest anchor

    except KeyboardInterrupt:
        print("\n\nInterrupted — sending STOP before exit ...")
        send_command(ser, "STOP")
        time.sleep(0.3)
        ser.close()
        print("Port closed.")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Toggle UWB TAG/STOP with automatic distance-threshold handover."
    )
    parser.add_argument(
        "--port", default=DEFAULT_PORT,
        help=f"Serial port (default: {DEFAULT_PORT})"
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD,
        help=f"Baud rate (default: {DEFAULT_BAUD})"
    )
    parser.add_argument(
        "--tag-duration", type=float, default=DEFAULT_TAG_DURATION,
        metavar="S",
        help=f"Max seconds in TAG mode before normal STOP (default: {DEFAULT_TAG_DURATION})"
    )
    parser.add_argument(
        "--stop-duration", type=float, default=DEFAULT_STOP_DURATION,
        metavar="S",
        help=f"Seconds in STOP/idle mode (default: {DEFAULT_STOP_DURATION})"
    )
    parser.add_argument(
        "--threshold", type=float, default=DEFAULT_DIST_THRESHOLD_CM,
        metavar="CM",
        help=f"Distance threshold in cm — sqrt(Xcm^2+Ycm^2) (default: {DEFAULT_DIST_THRESHOLD_CM})"
    )
    parser.add_argument(
        "--consec", type=int, default=DEFAULT_CONSEC_COUNT,
        metavar="N",
        help=f"Consecutive over-threshold readings before handover (default: {DEFAULT_CONSEC_COUNT})"
    )
    parser.add_argument(
        "--start-with", choices=["tag", "stop"], default="tag",
        help="First command sent on startup (default: tag)"
    )

    args = parser.parse_args()
    run(
        port=args.port,
        baud=args.baud,
        tag_duration=args.tag_duration,
        stop_duration=args.stop_duration,
        threshold_cm=args.threshold,
        consec_count=args.consec,
        start_with=args.start_with,
    )


if __name__ == "__main__":
    main()
