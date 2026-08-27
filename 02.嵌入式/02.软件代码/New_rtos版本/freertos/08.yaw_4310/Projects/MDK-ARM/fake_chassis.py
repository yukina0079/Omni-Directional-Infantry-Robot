#!/usr/bin/env python3
"""
Impersonate the chassis MCU on the yaw axis's serial link.

Why
---
The yaw board only moves when frames arrive from the chassis master, so the
receive path cannot be tested by staring at the board -- something has to talk
to it. This sends correctly-formed frames from the PC, which proves the whole
chain (DMA ring -> frame_sync -> command decode -> position loop) without
needing the real chassis MCU wired up.

With --show-reply it also decodes what the board sends back, which is the only
way to check the reply field's sign and range from outside the firmware.

Frame format (20 bytes), from 01.底盘主控/Drivers/BSP/DATA/data.c:

    0    0x55        header
    1-2  yaw angle   (the board's reply field; what it reports about itself)
    3
    4-5  lx stick
    6-7  ly stick
    8-9  rx stick
    10-11 ry stick
    12-13 chassis IMU yaw     -> yaw_num
    14-15 chassis yaw PID out -> yaw_pid_num   <- the actual command
    16-18
    19   0xFF        trailer

All 16-bit fields are signed, big-endian, scaled by 100.

Usage
-----
    python fake_chassis.py --cmd 0.3 --duration 5
    python fake_chassis.py --sweep --duration 20 --show-reply
"""

import argparse
import math
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial not installed: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def encode_signed(value: float) -> tuple[int, int]:
    """Float -> (high, low), matching float_to_two_uint8_signed()."""
    value = max(-327.68, min(327.67, value))
    scaled = int(round(value * 100.0))
    scaled &= 0xFFFF
    return (scaled >> 8) & 0xFF, scaled & 0xFF


def decode_signed(high: int, low: int) -> float:
    """(high, low) -> float, matching two_uint8_to_float_signed()."""
    raw = (high << 8) | low
    if raw >= 0x8000:
        raw -= 0x10000
    return raw / 100.0


def build_frame(yaw_pid: float, imu_yaw: float = 0.0) -> bytes:
    frame = bytearray(20)
    frame[0] = 0x55
    frame[19] = 0xFF
    frame[12], frame[13] = encode_signed(imu_yaw)
    frame[14], frame[15] = encode_signed(yaw_pid)
    return bytes(frame)


def parse_replies(buf: bytearray) -> list[float]:
    """
    Extract the reported angle from any complete reply frames in `buf`.

    Frames are located by the 0x55 ... 0xFF pattern rather than by assuming the
    stream is aligned, because the board's reply DMA is circular and
    free-running: we join it at an arbitrary offset, exactly as the real chassis
    does. Consumed bytes are removed from `buf` in place so the caller can keep
    appending to it.
    """
    angles: list[float] = []
    while True:
        start = buf.find(0x55)
        if start < 0:
            # No header at all: nothing here is useful, and keeping it would let
            # the buffer grow without bound.
            buf.clear()
            return angles
        if len(buf) - start < 20:
            del buf[:start]
            return angles
        if buf[start + 19] == 0xFF:
            angles.append(decode_signed(buf[start + 1], buf[start + 2]))
            del buf[:start + 20]
        else:
            # 0x55 was payload, not a header. Skip just past it and resync.
            del buf[:start + 1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--cmd", type=float, default=0.0,
                        help="constant yaw_pid_num command, radians")
    parser.add_argument("--sweep", action="store_true",
                        help="sweep the command as a slow sine instead")
    parser.add_argument("--amplitude", type=float, default=0.3)
    parser.add_argument("--period", type=float, default=6.0)
    parser.add_argument("--rate", type=float, default=100.0, help="frames/sec")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--show-reply", action="store_true",
                        help="decode and print the board's reply frames")
    parser.add_argument("--glitch-stats", action="store_true",
                        help="hunt for torn reply values (implies --show-reply)")
    parser.add_argument("--glitch-threshold", type=float, default=0.20,
                        help="rad; a jump larger than this between consecutive "
                             "replies cannot be real shaft motion")
    args = parser.parse_args()
    if args.glitch_stats:
        args.show_reply = True

    # DTR/RTS low: some USB-serial bridges drive the target's reset line from
    # these, and a reset mid-test would look exactly like a firmware crash.
    port = serial.Serial()
    port.port = args.port
    port.baudrate = args.baudrate
    port.timeout = 0            # non-blocking: we poll in_waiting instead
    port.dtr = False
    port.rts = False
    port.open()

    interval = 1.0 / args.rate
    start = time.time()
    sent = 0
    replies = 0
    rx = bytearray()
    last_report = start
    last_angle = None
    prev_reply = None
    max_jump = 0.0
    glitches: list[tuple[float, float, float, float]] = []

    print(f"sending frames to {args.port} at {args.rate:.0f} Hz for "
          f"{args.duration:.0f} s")
    if args.show_reply:
        print(f"{'t(s)':>6} {'cmd':>8} {'reply':>8}    "
              f"(reply = angle the board reports about itself)")

    try:
        while True:
            elapsed = time.time() - start
            if elapsed >= args.duration:
                break

            if args.sweep:
                cmd = args.amplitude * math.sin(2 * math.pi * elapsed / args.period)
            else:
                cmd = args.cmd

            port.write(build_frame(cmd))
            sent += 1

            if args.show_reply:
                waiting = port.in_waiting
                if waiting:
                    rx.extend(port.read(waiting))
                angles = parse_replies(rx)
                if angles:
                    replies += len(angles)
                    # Torn-frame detection. The reply DMA is circular and the
                    # firmware rewrites bytes 1/2 while it runs, so the receiver
                    # can in principle see a high byte from one sample paired
                    # with a low byte from the next. A real shaft cannot move
                    # more than a few mrad between consecutive frames, so any
                    # larger step is the tear, not motion.
                    for a in angles:
                        if prev_reply is not None:
                            jump = abs(a - prev_reply)
                            if jump > args.glitch_threshold:
                                glitches.append((elapsed, prev_reply, a, jump))
                            if jump > max_jump:
                                max_jump = jump
                        prev_reply = a
                    last_angle = angles[-1]
                if last_angle is not None and time.time() - last_report >= 0.5:
                    last_report = time.time()
                    print(f"{elapsed:>6.1f} {cmd:>+8.3f} {last_angle:>+8.3f}")

            time.sleep(interval)
    finally:
        port.close()

    print(f"sent {sent} frames in {time.time() - start:.1f} s")
    if args.show_reply:
        print(f"decoded {replies} reply frames")
    if args.glitch_stats:
        print(f"largest step between consecutive replies: {max_jump:.3f} rad "
              f"(threshold {args.glitch_threshold:.2f})")
        if glitches:
            rate = len(glitches) / replies * 100 if replies else 0.0
            print(f"TORN FRAMES: {len(glitches)} of {replies} ({rate:.3f}%)")
            for t, before, after, jump in glitches[:10]:
                print(f"  t={t:6.2f}s  {before:+.3f} -> {after:+.3f}  "
                      f"(jump {jump:+.3f})")
            if len(glitches) > 10:
                print(f"  ... and {len(glitches) - 10} more")
        else:
            print("no torn frames detected")
    return 0


if __name__ == "__main__":
    sys.exit(main())
