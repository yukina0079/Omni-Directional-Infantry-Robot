#!/usr/bin/env python3
"""
Live monitor for the yaw axis over SWD.

Why this exists
---------------
The board has exactly one usable UART (USART2, PA2/PA3). Once the chassis MCU
is wired to it, that wire carries the 20-byte binary protocol and printf() is no
longer usable for debugging -- debug text would corrupt the link and the link's
frames would bury the text. So the axis becomes un-observable at precisely the
moment it is in its real configuration.

SWD is already connected for flashing and reading target RAM does not disturb
the running CPU, so the debug channel moves there. This script polls the
g_yaw_monitor struct that data_change() refreshes every control cycle.

Usage
-----
    python yaw_monitor.py                 # poll until Ctrl-C
    python yaw_monitor.py --samples 20    # take 20 samples and stop
    python yaw_monitor.py --addr 0x20000064

The address defaults to whatever the .map file says; pass --addr to override,
and re-check it after any rebuild that changes RAM layout. The magic word guards
against reading a stale or wrong address -- if it does not match, the script
says so rather than printing plausible-looking garbage.
"""

import argparse
import json
import re
import struct
import subprocess
import sys
import time
from pathlib import Path

SKILL_DIR = Path.home() / ".claude" / "skills" / "openocd"
RUN_PY = SKILL_DIR / "scripts" / "openocd_run.py"
MAGIC = 0x59415731          # "YAW1"
STRUCT_FMT = "<7I6f"        # 7 uint32 then 6 float
STRUCT_LEN = struct.calcsize(STRUCT_FMT)

FLAG_NAMES = [
    (0x01, "ENERGISED"),
    (0x02, "ENC_FAULT"),
    (0x04, "COMMS_TIMEOUT"),
    (0x08, "OC_LATCHED"),
]


def find_map_address(project_root: Path) -> int | None:
    """Resolve g_yaw_monitor's address from the linker map."""
    for map_file in project_root.glob("**/*.map"):
        text = map_file.read_text(encoding="utf-8", errors="replace")
        match = re.search(r"g_yaw_monitor\s+(0x[0-9a-fA-F]+)", text)
        if match:
            return int(match.group(1), 16)
    return None


def read_memory(addr: int, words: int) -> list[int]:
    """Read `words` 32-bit words from target RAM via the openocd skill."""
    result = subprocess.run(
        [
            sys.executable, str(SKILL_DIR / "scripts" / "openocd_telnet.py"),
            "read-mem",
            "--address", hex(addr),
            "--length", str(words),
            "--width", "32",
            "--interface", "interface/cmsis-dap.cfg",
            "--target", "target/stm32f1x.cfg",
            "--adapter-speed", "1000",
            "--transport", "swd",
            "--json",
        ],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"openocd read failed: {result.stderr[:400]}")

    payload = json.loads(result.stdout)
    if payload.get("status") != "ok":
        raise RuntimeError(f"read-mem error: {payload.get('error')}")

    # The skill returns rows of space-separated hex words, one row per 32-byte
    # line, e.g. {"memory": [{"address": "0x...", "data": "59415731 0000..."}]}
    details = payload.get("details", {})
    rows = details.get("memory")
    if rows is None:
        raise RuntimeError(f"unexpected read-mem payload: {details}")

    values: list[int] = []
    for row in rows:
        values.extend(int(tok, 16) for tok in row["data"].split())

    if len(values) < words:
        raise RuntimeError(f"short read: wanted {words} words, got {len(values)}")

    return values[:words]


def decode(words: list[int]) -> dict:
    blob = b"".join(struct.pack("<I", w) for w in words)[:STRUCT_LEN]
    fields = struct.unpack(STRUCT_FMT, blob)
    magic, seq, loop_us, frames, oc_trips, flags, reply_skips = fields[:7]
    cmd, angle, err, vel, iq, yaw_pid = fields[7:]
    return {
        "magic": magic, "seq": seq, "loop_us": loop_us, "frames": frames,
        "oc_trips": oc_trips, "flags": flags, "reply_skips": reply_skips,
        "cmd": cmd, "angle": angle,
        "err": err, "vel": vel, "iq": iq, "yaw_pid": yaw_pid,
    }


def flag_text(flags: int) -> str:
    names = [name for bit, name in FLAG_NAMES if flags & bit]
    return ",".join(names) if names else "idle"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--addr", type=lambda s: int(s, 0), default=None)
    parser.add_argument("--samples", type=int, default=0, help="0 = forever")
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[2]
    addr = args.addr or find_map_address(root)
    if addr is None:
        print("could not resolve g_yaw_monitor address; pass --addr", file=sys.stderr)
        return 1

    print(f"monitoring g_yaw_monitor at {addr:#010x}")
    print(f"{'seq':>8} {'loop':>6} {'frames':>7} {'cmd':>7} {'angle':>7} "
          f"{'err':>7} {'vel':>7} {'Iq(mA)':>7}  state")

    taken = 0
    prev_seq = prev_frames = prev_skips = None
    while args.samples == 0 or taken < args.samples:
        try:
            snap = decode(read_memory(addr, STRUCT_LEN // 4))
        except Exception as exc:                       # noqa: BLE001
            print(f"read error: {exc}", file=sys.stderr)
            return 1

        if snap["magic"] != MAGIC:
            print(f"MAGIC MISMATCH: got {snap['magic']:#010x}, expected "
                  f"{MAGIC:#010x}. Address is wrong or firmware is stale.",
                  file=sys.stderr)
            return 1

        # Rates are more informative than absolute counts: a frozen seq means
        # the control loop has stopped, and a frozen frame count means the
        # chassis link is dead even though the loop is still running.
        #
        # The skip percentage is a health check on the P1-9 fix: reply_publish()
        # defers the update whenever the TX DMA is mid-way through the angle
        # field, which should be roughly 3 bytes out of 20. A figure near 0%
        # would mean the DMA is not running (so the guard never fires and the
        # reply is stale); near 100% would mean the window never opens.
        rate_note = ""
        if prev_seq is not None:
            d_seq = snap["seq"] - prev_seq
            d_frames = snap["frames"] - prev_frames
            d_skips = snap["reply_skips"] - prev_skips
            rate_note = f"  +{d_seq} cyc, +{d_frames} frm"
            if d_seq:
                rate_note += f", skip {100.0 * d_skips / d_seq:.0f}%"
            if d_seq == 0:
                rate_note += "  <-- LOOP STALLED"
        prev_seq, prev_frames = snap["seq"], snap["frames"]
        prev_skips = snap["reply_skips"]

        print(f"{snap['seq']:>8} {snap['loop_us']:>5}u {snap['frames']:>7} "
              f"{snap['cmd']:>+7.3f} {snap['angle']:>+7.3f} {snap['err']:>+7.3f} "
              f"{snap['vel']:>+7.2f} {snap['iq']*1000:>7.0f}  "
              f"{flag_text(snap['flags'])}{rate_note}")

        taken += 1
        if args.samples == 0 or taken < args.samples:
            time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    sys.exit(main())
