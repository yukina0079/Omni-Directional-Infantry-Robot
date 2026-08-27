#!/usr/bin/env python3
"""Capture g_yaw_monitor over SWD without resetting the target."""

from __future__ import annotations

import argparse
import csv
import re
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

OPENOCD = (
    r"C:\Users\35252\AppData\Local\Microsoft\WinGet\Packages"
    r"\xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe"
    r"\xpack-openocd-0.12.0-7\bin\openocd.exe"
)
MAGIC = 0x59415731
MON_WORDS = 13
FIELDS = (
    "magic", "seq", "loop_us", "frames", "oc_trips", "flags",
    "reply_skips", "cmd", "angle", "err", "vel", "iq", "yaw_pid",
)


def find_addr() -> int:
    root = Path(__file__).resolve().parents[2]
    for map_file in root.glob("**/*.map"):
        text = map_file.read_text(encoding="utf-8", errors="replace")
        match = re.search(r"g_yaw_monitor\s+(0x[0-9a-fA-F]+)", text)
        if match:
            return int(match.group(1), 16)
    raise SystemExit("g_yaw_monitor not found in map")


def start_openocd() -> subprocess.Popen:
    return subprocess.Popen(
        [
            OPENOCD,
            "-f", "interface/cmsis-dap.cfg",
            "-f", "target/stm32f1x.cfg",
            "-c", "adapter speed 100",
            "-c", "transport select swd",
            "-c", "reset_config none",
            "-c", "gdb port 3333",
            "-c", "telnet port 4444",
            "-c", "init",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def wait_telnet(timeout: float = 8.0) -> socket.socket:
    deadline = time.time() + timeout
    last_err = None
    while time.time() < deadline:
        try:
            sock = socket.create_connection(("127.0.0.1", 4444), timeout=1.0)
            sock.settimeout(2.0)
            # banner
            try:
                sock.recv(4096)
            except socket.timeout:
                pass
            return sock
        except OSError as exc:
            last_err = exc
            time.sleep(0.1)
    raise SystemExit(f"OpenOCD telnet not ready: {last_err}")


def ocd(sock: socket.socket, cmd: str) -> str:
    sock.sendall((cmd + "\n").encode("ascii"))
    chunks = []
    while True:
        try:
            data = sock.recv(4096)
        except socket.timeout:
            break
        if not data:
            break
        chunks.append(data.decode("utf-8", "replace"))
        if "> " in chunks[-1]:
            break
    return "".join(chunks)


def read_monitor(sock: socket.socket, addr: int) -> dict | None:
    text = ocd(sock, f"mdw 0x{addr:08x} {MON_WORDS}").replace("\x00", "")
    words: list[int] = []
    for line in text.splitlines():
        match = re.search(r"0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{8}\s*)+)", line.strip())
        if match:
            words.extend(int(tok, 16) for tok in match.group(1).split())
    if len(words) < MON_WORDS:
        return None
    blob = b"".join(struct.pack("<I", w) for w in words[:MON_WORDS])
    values = struct.unpack("<7I6f", blob)
    return dict(zip(FIELDS, values))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=22.0)
    ap.add_argument("--hz", type=float, default=40.0)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    addr = find_addr()
    proc = start_openocd()
    try:
        sock = wait_telnet()
        time.sleep(0.4)
        first = None
        raw = ""
        for _ in range(8):
            raw = ocd(sock, f"mdw 0x{addr:08x} {MON_WORDS}").replace("\x00", "")
            words: list[int] = []
            for line in raw.splitlines():
                match = re.search(
                    r"0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{8}\s*)+)",
                    line.strip(),
                )
                if match:
                    words.extend(int(tok, 16) for tok in match.group(1).split())
            if len(words) >= MON_WORDS:
                blob = b"".join(struct.pack("<I", w) for w in words[:MON_WORDS])
                first = dict(zip(FIELDS, struct.unpack("<7I6f", blob)))
                break
            time.sleep(0.2)
        if first is None:
            print("first read failed, raw=", raw[:500], file=sys.stderr)
            return 1
        if first["magic"] != MAGIC:
            print(
                f"MAGIC MISMATCH {first['magic']:#010x} at {addr:#010x}",
                file=sys.stderr,
            )
            return 1
        print(f"RECORDING {args.seconds:.0f}s @ {args.hz:.0f} Hz  addr={addr:#010x}  TWIST NOW", flush=True)

        interval = 1.0 / args.hz
        t0 = time.perf_counter()
        rows = []
        while True:
            now = time.perf_counter()
            elapsed = now - t0
            if elapsed >= args.seconds:
                break
            snap = read_monitor(sock, addr)
            if snap and snap["magic"] == MAGIC:
                snap["t"] = elapsed
                rows.append(snap)
            remain = interval - (time.perf_counter() - now)
            if remain > 0:
                time.sleep(remain)

        ocd(sock, "exit")
        sock.close()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=["t", *FIELDS])
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row[k] for k in ["t", *FIELDS]})

    print(f"wrote {len(rows)} samples -> {args.out}", flush=True)
    return 0 if rows else 1


if __name__ == "__main__":
    sys.exit(main())
