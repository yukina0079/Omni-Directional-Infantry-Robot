#!/usr/bin/env python3
"""
Sample the chassis link state over SWD from ONE OpenOCD session.

Shows the stick command as the chassis computed it, the two bytes it actually
transmits, and the yaw board's decoded reply -- so a mismatch between "what the
stick says" and "what went on the wire" is visible directly. That mismatch is
what a clamp used to hide.

One server, chained sleep/mdw, no halt between samples: repeated openocd
invocations each halt the target and never resume, which freezes every sample
after the first.

Usage:
    python chassis_watch.py [--seconds 14] [--interval 0.5]
"""

import argparse
import re
import struct
import subprocess
import sys

OPENOCD = (r"C:\Users\35252\AppData\Local\Microsoft\WinGet\Packages"
           r"\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe"
           r"\xpack-openocd-0.12.0-7\bin\openocd.exe")

JOY_YAW_NUM = 0x20000074       # float, stick angle command (radians)
GIMBLE_YAW = 0x200000a0        # float, decoded reply from the yaw board
TX_BUF = 0x200003c4            # uint8[20], frame sent to the yaw board
RX_BUF = 0x200003d8            # uint8[20], frame received from it


def s16(hi: int, lo: int) -> int:
    v = ((hi & 0xFF) << 8) | (lo & 0xFF)
    return v - 0x10000 if v & 0x8000 else v


def build_cmds(samples: int, interval_ms: int) -> list[str]:
    cmds = ["-c", "adapter speed 1000", "-c", "transport select swd",
            "-c", "init", "-c", "reset run"]
    for i in range(samples):
        cmds += ["-c", f"sleep {interval_ms}",
                 "-c", f"echo \"--SAMPLE {i}--\"",
                 "-c", f"mdw {JOY_YAW_NUM:#x} 1",
                 "-c", f"mdw {GIMBLE_YAW:#x} 1",
                 "-c", f"mdb {TX_BUF:#x} 20",
                 "-c", f"mdb {RX_BUF:#x} 20"]
    cmds += ["-c", "shutdown"]
    return cmds


def parse(text: str) -> list[list[str]]:
    groups, current = [], None
    for line in text.splitlines():
        if "--SAMPLE" in line:
            if current:
                groups.append(current)
            current = []
            continue
        if current is not None and re.match(r"^0x[0-9a-fA-F]+:", line.strip()):
            current.append(line.strip())
    if current:
        groups.append(current)
    return groups


def decode(rows: list[str]) -> dict | None:
    """Rows arrive in issue order: joy word, gimble word, 20 TX bytes, 20 RX."""
    words, byts = [], []
    for row in rows:
        toks = row.split(":", 1)[1].split()
        if all(len(t) == 8 for t in toks):
            words.extend(int(t, 16) for t in toks)
        else:
            byts.extend(int(t, 16) for t in toks)
    if len(words) < 2 or len(byts) < 40:
        return None
    joy = struct.unpack("<f", struct.pack("<I", words[0]))[0]
    gim = struct.unpack("<f", struct.pack("<I", words[1]))[0]
    tx, rx = byts[:20], byts[20:40]
    return {"joy": joy, "gimble": gim, "tx": tx, "rx": rx,
            "tx_cmd": s16(tx[14], tx[15]) / 100.0,
            "tx_imu": s16(tx[12], tx[13]) / 100.0,
            "rx_angle": s16(rx[1], rx[2]) / 100.0}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=14.0)
    ap.add_argument("--interval", type=float, default=0.5)
    args = ap.parse_args()

    n = max(1, int(args.seconds / args.interval))
    proc = subprocess.run(
        [OPENOCD, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32f4x.cfg"]
        + build_cmds(n, int(args.interval * 1000)),
        capture_output=True, text=True, timeout=args.seconds + 60)
    text = proc.stdout + proc.stderr

    samples = [s for s in (decode(g) for g in parse(text)) if s]
    if not samples:
        print("no samples parsed. raw tail:\n" + text[-1500:], file=sys.stderr)
        return 1

    print(f"{'t':>5} {'joy_yaw':>8} {'tx_cmd':>8} {'tx_imu':>7} "
          f"{'rx_angle':>9} {'gimble':>8}  frame")
    for i, s in enumerate(samples):
        ok = s["tx"][0] == 0x55 and s["tx"][19] == 0xFF
        rok = s["rx"][0] == 0x55 and s["rx"][19] == 0xFF
        mark = ("tx ok" if ok else "TX BAD") + "/" + ("rx ok" if rok else "RX BAD")
        # tx_cmd must equal joy_yaw once the clamp is gone; flag any divergence
        # beyond the protocol's own 0.01 rad quantisation.
        if abs(s["tx_cmd"] - s["joy"]) > 0.02:
            mark += f"  <-- TX != joy (dropped {s['joy'] - s['tx_cmd']:+.2f})"
        print(f"{i * args.interval:>5.1f} {s['joy']:>+8.3f} {s['tx_cmd']:>+8.2f} "
              f"{s['tx_imu']:>+7.2f} {s['rx_angle']:>+9.2f} "
              f"{s['gimble']:>+8.3f}  {mark}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
