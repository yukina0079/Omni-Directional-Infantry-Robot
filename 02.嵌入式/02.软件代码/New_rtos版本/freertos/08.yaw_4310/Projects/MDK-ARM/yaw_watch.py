#!/usr/bin/env python3
"""
Sample g_yaw_monitor over SWD from ONE OpenOCD session.

Why not yaw_monitor.py: that script shells out to the openocd skill once per
sample, and each invocation starts a fresh server that halts the target and
never resumes it. Every sample after the first therefore returns the same
frozen snapshot -- the axis looks dead even when it is running fine. Here a
single server is started once, `reset run` lets the target go, and the reads
are chained with `sleep` in between so the CPU keeps running throughout.

Usage:
    python yaw_watch.py [--seconds 12] [--interval 0.6]
"""

import argparse
import re
import struct
import subprocess
import sys

OPENOCD = (r"C:\Users\35252\AppData\Local\Microsoft\WinGet\Packages"
           r"\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe"
           r"\xpack-openocd-0.12.0-7\bin\openocd.exe")

MON_ADDR = 0x20000068          # g_yaw_monitor, from stm32f103.map
MON_WORDS = 13                 # 7 uint32 + 6 float
MAGIC = 0x59415731             # "YAW1"

FLAG_NAMES = [(0x01, "ENERGISED"), (0x02, "ENC_FAULT"),
              (0x04, "COMMS_TIMEOUT"), (0x08, "OC_LATCHED")]


def build_cmds(samples: int, interval_ms: int) -> list[str]:
    cmds = ["-c", "adapter speed 1000", "-c", "transport select swd",
            "-c", "init", "-c", "reset run"]
    for i in range(samples):
        cmds += ["-c", f"sleep {interval_ms}",
                 "-c", f"echo \"--SAMPLE {i}--\"",
                 "-c", f"mdw {MON_ADDR:#x} {MON_WORDS}"]
    cmds += ["-c", "shutdown"]
    return cmds


def parse(text: str) -> list[list[int]]:
    """Group `mdw` output rows into one word-list per --SAMPLE-- marker."""
    samples, current = [], None
    row_re = re.compile(r"^0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{8}\s*)+)")
    for line in text.splitlines():
        if "--SAMPLE" in line:
            if current:
                samples.append(current)
            current = []
            continue
        m = row_re.match(line.strip())
        if m is not None and current is not None:
            current.extend(int(tok, 16) for tok in m.group(1).split())
    if current:
        samples.append(current)
    return samples


def decode(words: list[int]) -> dict | None:
    if len(words) < MON_WORDS:
        return None
    blob = b"".join(struct.pack("<I", w) for w in words[:MON_WORDS])
    f = struct.unpack("<7I6f", blob)
    return dict(zip(
        ("magic", "seq", "loop_us", "frames", "oc_trips", "flags",
         "reply_skips", "cmd", "angle", "err", "vel", "iq", "yaw_pid"), f))


def flag_text(flags: int) -> str:
    names = [n for bit, n in FLAG_NAMES if flags & bit]
    return ",".join(names) if names else "idle"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--interval", type=float, default=0.6)
    args = ap.parse_args()

    n = max(1, int(args.seconds / args.interval))
    proc = subprocess.run(
        [OPENOCD, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32f1x.cfg"]
        + build_cmds(n, int(args.interval * 1000)),
        capture_output=True, text=True, timeout=args.seconds + 60,
    )
    # OpenOCD writes its log to stderr, mdw output included.
    text = proc.stdout + proc.stderr

    samples = [decode(w) for w in parse(text)]
    samples = [s for s in samples if s]
    if not samples:
        print("no samples parsed. raw tail:\n" + text[-1500:], file=sys.stderr)
        return 1

    if samples[0]["magic"] != MAGIC:
        print(f"MAGIC MISMATCH: {samples[0]['magic']:#010x} != {MAGIC:#010x}"
              " -- wrong address or stale firmware", file=sys.stderr)
        return 1

    print(f"{'t':>5} {'seq':>9} {'loop':>6} {'frames':>7} {'cmd':>7} "
          f"{'angle':>7} {'err':>7} {'vel':>7} {'Iq(mA)':>7}  state")
    prev = None
    for i, s in enumerate(samples):
        note = ""
        if prev:
            d_seq = s["seq"] - prev["seq"]
            d_frm = s["frames"] - prev["frames"]
            note = f"  +{d_seq} cyc, +{d_frm} frm"
            if d_seq == 0:
                note += "  <-- LOOP STALLED"
            if d_frm == 0:
                note += "  <-- NO FRAMES"
        prev = s
        print(f"{i * args.interval:>5.1f} {s['seq']:>9} {s['loop_us']:>5}u "
              f"{s['frames']:>7} {s['cmd']:>+7.3f} {s['angle']:>+7.3f} "
              f"{s['err']:>+7.3f} {s['vel']:>+7.2f} {s['iq'] * 1000:>7.0f}  "
              f"{flag_text(s['flags'])}{note}")

    print(f"\noc_trips={samples[-1]['oc_trips']} "
          f"reply_skips={samples[-1]['reply_skips']} "
          f"yaw_pid_num={samples[-1]['yaw_pid']:+.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
