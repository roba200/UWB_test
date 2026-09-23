"""
binary_uart_monitor.py
Monitors and decodes the UWB firmware's binary UART protocol (reportLevel==3),
as defined in NRF_Juniper/twr_demo/Src/Inc/binary_uart_protocol.h.

Recognizes every frame type defined there:
    0xAA  Legacy TWR frame            (15 bytes)
    0xAE  Extended TWR frame          (27 bytes; flags bit0=stationary, bit1=escort)
    0xAB  New-tag frame               (11 bytes)
    0xAC  EUI request frame           (11 bytes)
    0xAD  EUI response frame          (11 bytes)
    0xAF  Tag-location frame          (14 bytes)  (Tag's own "TAG" report)

Any bytes that are not part of a valid, checksummed binary frame are treated
as plain text (JSON at reportLevel 1, "RA ..." lines at reportLevel 2, or
command replies like "ok\\r\\n"/"error ...") and printed as-is, so this works
regardless of the current reportLevel or if plain command replies are mixed
into the stream.

Usage:
    python binary_uart_monitor.py --port COM35
    python binary_uart_monitor.py --port COM35 --baud 115200
    python binary_uart_monitor.py --port COM35 --log capture.csv
"""

import argparse
import csv
import struct
import sys
import time
from dataclasses import dataclass
from typing import Callable, Optional

import serial


# ── defaults ──────────────────────────────────────────────────────────────────
DEFAULT_PORT = "COM35"
DEFAULT_BAUD = 115200
TICK_IDLE = 0.001  # sleep only when RX buffer is empty (seconds)

BIN_FRAME_END = 0x55


# ── frame decoding ───────────────────────────────────────────────────────────

@dataclass
class FrameSpec:
    name: str
    size: int
    struct_fmt: str          # includes leading '<' and the start-byte field
    field_names: tuple
    payload_slice: slice     # bytes covered by the XOR checksum


def _xor_checksum(payload: bytes) -> int:
    c = 0
    for b in payload:
        c ^= b
    return c


def _decode_legacy_twr(fields: dict) -> str:
    return (f"[TWR      ] tag=0x{fields['tag_id']:04X} anchor=0x{fields['anchor_id']:04X} "
            f"dist={fields['distance_cm']:6d} cm  R={fields['range_num']:3d}  "
            f"flags=0x{fields['flags']:02X}  X={fields['x_cm']:6d}cm Y={fields['y_cm']:6d}cm")


BIN_EXT_FLAG_STATIONARY = 0x01
BIN_EXT_FLAG_ESCORT     = 0x02


def _decode_ext_twr(fields: dict) -> str:
    hr = fields['heart_rate_x10']
    spo2 = fields['spo2_x10']
    hr_str = f"{hr / 10.0:5.1f}bpm" if hr else "  --  "
    spo2_str = f"{spo2 / 10.0:5.1f}%" if spo2 else " -- "
    flags = fields['flags']
    stationary = "STATIONARY" if flags & BIN_EXT_FLAG_STATIONARY else "          "
    escort = "ESCORT" if flags & BIN_EXT_FLAG_ESCORT else "      "
    return (f"[TWR-EXT  ] tag=0x{fields['tag_id']:04X} anchor=0x{fields['anchor_id']:04X} "
            f"dist={fields['distance_mm']:7d} mm  R={fields['range_num']:3d}  "
            f"flags=0x{flags:04X} [{stationary} {escort}]  "
            f"X={fields['x_cm']:6d}cm Y={fields['y_cm']:6d}cm "
            f"pdoa={fields['pdoa_cdeg'] / 100.0:7.2f} deg  "
            f"bat={fields['battery_percent']:3d}% HR={hr_str} SpO2={spo2_str} "
            f"temp={fields['temp_centi_c'] / 100.0:5.2f}C")


def _decode_new_tag(fields: dict) -> str:
    return f"[NEW-TAG  ] addr64=0x{fields['tag_addr64']:016X}"


def _decode_eui_req(fields: dict) -> str:
    return f"[EUI-REQ  ] cmd=0x{fields['cmd']:02X}"


def _decode_eui_resp(fields: dict) -> str:
    return f"[EUI-RESP ] eui64=0x{fields['eui64']:016X}"


def _decode_tag_loc(fields: dict) -> str:
    return (f"[TAG-LOC  ] tag=0x{fields['tag_addr']:04X} node=0x{fields['node_addr']:04X} "
            f"R={fields['range_num']:3d}  X={fields['x_cm']:6d}cm Y={fields['y_cm']:6d}cm "
            f"clkOff={fields['clk_offset_pphm']:6d} (0.01ppm)")


# start_byte -> (FrameSpec, decode_fn)
FRAME_TABLE = {
    0xAA: (
        FrameSpec("TWR", 15, "<BHHHBBhhBB",
                  ("start", "tag_id", "anchor_id", "distance_cm", "range_num",
                   "flags", "x_cm", "y_cm", "checksum", "end"),
                  slice(1, 13)),
        _decode_legacy_twr,
    ),
    0xAE: (
        # Grew from 20 to 27 bytes: battery_percent/heart_rate_x10/spo2_x10/
        # temp_centi_c (the tag's host-MCU vitals, from AT+SENS) were added
        # before the checksum - see binary_uart_protocol.h's
        # binary_ext_twr_frame_t.
        FrameSpec("TWR-EXT", 27, "<BHHIBHhhhBHHhBB",
                  ("start", "tag_id", "anchor_id", "distance_mm", "range_num",
                   "flags", "x_cm", "y_cm", "pdoa_cdeg",
                   "battery_percent", "heart_rate_x10", "spo2_x10", "temp_centi_c",
                   "checksum", "end"),
                  slice(1, 25)),
        _decode_ext_twr,
    ),
    0xAB: (
        FrameSpec("NEW-TAG", 11, "<BQBB",
                  ("start", "tag_addr64", "checksum", "end"),
                  slice(1, 9)),
        _decode_new_tag,
    ),
    0xAC: (
        FrameSpec("EUI-REQ", 11, "<BB7sBB",
                  ("start", "cmd", "reserved", "checksum", "end"),
                  slice(1, 9)),
        _decode_eui_req,
    ),
    0xAD: (
        FrameSpec("EUI-RESP", 11, "<BQBB",
                  ("start", "eui64", "checksum", "end"),
                  slice(1, 9)),
        _decode_eui_resp,
    ),
    0xAF: (
        FrameSpec("TAG-LOC", 14, "<BHHBhhhBB",
                  ("start", "tag_addr", "node_addr", "range_num",
                   "x_cm", "y_cm", "clk_offset_pphm", "checksum", "end"),
                  slice(1, 12)),
        _decode_tag_loc,
    ),
}


def try_decode_frame(buf: bytes) -> Optional[dict]:
    """
    buf[0] must be a known start byte and len(buf) >= that frame's size.
    Returns a dict with 'name', 'size', 'text' (pretty string) and 'fields'
    if buf[:size] is a structurally valid, checksummed frame - else None.
    """
    spec, decode_fn = FRAME_TABLE[buf[0]]
    frame = buf[:spec.size]

    if frame[-1] != BIN_FRAME_END:
        return None

    expected_checksum = _xor_checksum(frame[spec.payload_slice])
    if expected_checksum != frame[-2]:
        return None

    values = struct.unpack(spec.struct_fmt, frame)
    fields = dict(zip(spec.field_names, values))

    return {
        "name": spec.name,
        "size": spec.size,
        "text": decode_fn(fields),
        "fields": fields,
        "raw": frame,
    }


# ── stream processing ─────────────────────────────────────────────────────────

class StreamMonitor:
    """
    Feed raw serial bytes in via feed(); emits decoded binary frames and
    plain-text lines via the on_frame/on_text callbacks as they complete.
    Handles frame/text bytes interleaved in the same stream, and resyncs
    byte-by-byte if a byte that happens to match a known start marker turns
    out not to be a real, validly-checksummed frame.
    """

    def __init__(self, on_frame: Callable[[dict], None], on_text: Callable[[str], None]):
        self._buf = bytearray()
        self._text_buf = bytearray()
        self._on_frame = on_frame
        self._on_text = on_text

    def feed(self, data: bytes) -> None:
        self._buf.extend(data)
        self._process()

    def _process(self) -> None:
        while self._buf:
            b0 = self._buf[0]

            if b0 in FRAME_TABLE:
                spec, _ = FRAME_TABLE[b0]
                if len(self._buf) < spec.size:
                    return  # wait for more bytes before deciding

                decoded = try_decode_frame(bytes(self._buf[:spec.size]))
                if decoded is not None:
                    self._flush_text()
                    self._on_frame(decoded)
                    del self._buf[:spec.size]
                    continue
                # False positive (a text byte happened to match a start
                # marker) - fall through and treat b0 as plain text below.

            self._text_buf.append(self._buf[0])
            del self._buf[0]
            if self._text_buf and self._text_buf[-1] == 0x0A:  # '\n'
                self._flush_text()

    def _flush_text(self) -> None:
        if not self._text_buf:
            return
        line = self._text_buf.decode("ascii", errors="replace").rstrip("\r\n")
        self._text_buf.clear()
        if line.strip():
            self._on_text(line)

    def flush(self) -> None:
        self._flush_text()


# ── main loop ─────────────────────────────────────────────────────────────────

def run(port: str, baud: int, log_path: Optional[str]) -> None:
    print(f"Opening {port} at {baud} baud ...")
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,  # non-blocking reads — poll in a loop
        )
    except serial.SerialException as exc:
        sys.exit(f"Cannot open port: {exc}")

    log_file = None
    log_writer = None
    if log_path:
        log_file = open(log_path, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["timestamp", "type", "fields"])
        print(f"Logging decoded frames to {log_path}")

    counts = {}

    def on_frame(decoded: dict) -> None:
        ts = time.strftime("%H:%M:%S")
        print(f"[{ts}] {decoded['text']}")
        counts[decoded["name"]] = counts.get(decoded["name"], 0) + 1
        if log_writer:
            log_writer.writerow([ts, decoded["name"], decoded["fields"]])

    def on_text(line: str) -> None:
        ts = time.strftime("%H:%M:%S")
        print(f"[{ts}]   .. {line}")

    monitor = StreamMonitor(on_frame, on_text)

    print("Press Ctrl-C to quit.\n")
    try:
        while True:
            n = ser.in_waiting
            if n:
                monitor.feed(ser.read(n))
            else:
                time.sleep(TICK_IDLE)
    except KeyboardInterrupt:
        monitor.flush()
        print("\n\nInterrupted.")
        if counts:
            print("Frame counts:")
            for name, n in sorted(counts.items()):
                print(f"  {name:10s} {n}")
    finally:
        ser.close()
        if log_file:
            log_file.close()
        print("Port closed.")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Monitor and decode the UWB firmware's binary UART protocol."
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
        "--log", default=None, metavar="FILE.csv",
        help="Optional CSV file to log decoded frames to"
    )

    args = parser.parse_args()
    run(port=args.port, baud=args.baud, log_path=args.log)


if __name__ == "__main__":
    main()
