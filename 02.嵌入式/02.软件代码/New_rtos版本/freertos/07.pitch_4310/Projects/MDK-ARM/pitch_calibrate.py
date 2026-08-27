#!/usr/bin/env python3
"""Hand-sweep pitch travel from the AS5600 over SWD.

Attaches without reset so Motor_init does not twitch the barrel.
Polls g_pitch_monitor and writes a live JSON snapshot the host can read.

    py -3.13 pitch_calibrate.py
    py -3.13 pitch_calibrate.py --seconds 90
"""

from __future__ import annotations

import argparse
import json
import math
import re
import struct
import sys
import time
from pathlib import Path

MAGIC = 0x50495431  # "PIT1"
# 7 uint32 + 9 float + 1 uint32  (see Data.c pitch_monitor_t)
STRUCT_FMT = "<7I9fI"
STRUCT_LEN = struct.calcsize(STRUCT_FMT)
DEFAULT_ADDR = 0x20000088
RAD2DEG = 180.0 / math.pi

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
STATUS_PATH = HERE / "pitch_cal_status.json"
MAP_PATH = ROOT / "Output" / "stm32f103.map"


def find_addr() -> int:
    if MAP_PATH.is_file():
        text = MAP_PATH.read_text(encoding="utf-8", errors="replace")
        match = re.search(r"g_pitch_monitor\s+(0x[0-9a-fA-F]+)", text)
        if match:
            return int(match.group(1), 16)
    return DEFAULT_ADDR


def decode(raw: bytes) -> dict | None:
    if len(raw) < STRUCT_LEN:
        return None
    vals = struct.unpack(STRUCT_FMT, raw[:STRUCT_LEN])
    magic = vals[0]
    if magic != MAGIC:
        return None
    return {
        "magic": magic,
        "seq": vals[1],
        "loop_us": vals[2],
        "frames": vals[3],
        "oc_trips": vals[4],
        "flags": vals[5],
        "reply_skips": vals[6],
        "cmd": vals[7],
        "angle": vals[8],
        "err": vals[9],
        "vel": vals[10],
        "iq": vals[11],
        "pid": vals[12],
        "angle_abs": vals[13],
        "cal_min": vals[14],
        "cal_max": vals[15],
        "raw": vals[16],
    }


def write_status(payload: dict) -> None:
    tmp = STATUS_PATH.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    tmp.replace(STATUS_PATH)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--addr", type=lambda s: int(s, 0), default=None)
    parser.add_argument("--hz", type=float, default=15.0)
    parser.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    args = parser.parse_args()

    addr = args.addr or find_addr()
    period = 1.0 / max(args.hz, 1.0)

    from pyocd.core.helpers import ConnectHelper

    print(f"attach g_pitch_monitor @ {addr:#010x}  (no reset)")
    session = ConnectHelper.session_with_chosen_probe(
        target_override="cortex_m",
        connect_mode="attach",
        options={"frequency": 400000, "auto_unlock": False},
    )
    if session is None:
        print("no ST-Link", file=sys.stderr)
        return 1
    session.open()
    target = session.target

    words_n = (STRUCT_LEN + 3) // 4
    host_min = None
    host_max = None
    t0 = time.monotonic()
    last_print = 0.0

    try:
        while True:
            now = time.monotonic()
            if args.seconds > 0.0 and (now - t0) >= args.seconds:
                break
            try:
                words = target.read_memory_block32(addr, words_n)
                raw = b"".join(int(w).to_bytes(4, "little") for w in words)[:STRUCT_LEN]
                snap = decode(raw)
            except Exception as exc:  # noqa: BLE001
                write_status({"ok": False, "error": str(exc)})
                print(f"read fail: {exc}", file=sys.stderr)
                time.sleep(0.4)
                continue

            if snap is None:
                magic = struct.unpack_from("<I", raw)[0] if len(raw) >= 4 else 0
                write_status({"ok": False, "error": f"bad magic 0x{magic:08X}"})
                print(f"bad magic 0x{magic:08X} — flash the new pitch firmware first")
                time.sleep(0.5)
                continue

            angle = snap["angle"]
            if host_min is None or angle < host_min:
                host_min = angle
            if host_max is None or angle > host_max:
                host_max = angle

            payload = {
                "ok": True,
                "t": now - t0,
                "seq": snap["seq"],
                "flags": snap["flags"],
                "frames": snap["frames"],
                "raw": snap["raw"],
                "angle_rad": angle,
                "angle_deg": angle * RAD2DEG,
                "angle_abs_rad": snap["angle_abs"],
                "angle_abs_deg": snap["angle_abs"] * RAD2DEG,
                "fw_min_rad": snap["cal_min"],
                "fw_max_rad": snap["cal_max"],
                "host_min_rad": host_min,
                "host_max_rad": host_max,
                "travel_deg": (host_max - host_min) * RAD2DEG,
                "vel": snap["vel"],
                "iq_mA": snap["iq"] * 1000.0,
            }
            write_status(payload)

            if (now - last_print) >= 0.25:
                last_print = now
                print(
                    f"raw={snap['raw']:4d}  ang={angle:+7.3f}rad "
                    f"({angle * RAD2DEG:+6.1f}deg)  "
                    f"abs={snap['angle_abs']:+6.3f}  "
                    f"min={host_min:+7.3f} max={host_max:+7.3f}  "
                    f"span={(host_max - host_min) * RAD2DEG:5.1f}deg  "
                    f"flags=0x{snap['flags']:X} frm={snap['frames']}"
                )

            leftover = period - (time.monotonic() - now)
            if leftover > 0:
                time.sleep(leftover)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        try:
            session.close()
        except Exception:
            pass

    if STATUS_PATH.is_file():
        print(f"last snapshot: {STATUS_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
