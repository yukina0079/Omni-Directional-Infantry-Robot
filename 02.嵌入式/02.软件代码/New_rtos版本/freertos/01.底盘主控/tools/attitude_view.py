#!/usr/bin/env python3
"""Live chassis attitude viewer.

ST-Link SRAM polling runs in a background thread. The matplotlib window
only draws the last good sample, so a hung SWD read cannot freeze the UI.

Target VCC on this board often reads ~0.4 V on the ST-Link sense pin.
Keep SWD slow (400 kHz) and poll at 10 Hz.
"""

from __future__ import annotations

import argparse
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np

TELEM_FMT = "<IffffffffffBB"
TELEM_SIZE = struct.calcsize(TELEM_FMT)
TELEM_MAGIC = 0x41545431
DEFAULT_ADDR = 0x20000424


def find_telem_addr(map_path: Path) -> int:
    if map_path.is_file():
        for line in map_path.read_text(encoding="utf-8", errors="ignore").splitlines():
            if "imu_telem" in line and "0x20" in line:
                for tok in line.split():
                    if tok.startswith("0x20"):
                        return int(tok, 16)
    return DEFAULT_ADDR


def rot_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def chassis_faces():
    hx, hy, hz = 1.2, 0.7, 0.25
    verts = np.array(
        [
            [hx, hy, -hz],
            [hx, -hy, -hz],
            [-hx, -hy, -hz],
            [-hx, hy, -hz],
            [hx, hy, hz],
            [hx, -hy, hz],
            [-hx, -hy, hz],
            [-hx, hy, hz],
        ],
        dtype=float,
    )
    faces = [
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [3, 0, 4, 7],
        [1, 2, 6, 5],
        [4, 5, 6, 7],
        [0, 3, 2, 1],
    ]
    colors = ["#e74c3c", "#7f8c8d", "#3498db", "#2ecc71", "#f1c40f", "#34495e"]
    return verts, faces, colors


def parse_telem(raw: bytes) -> dict | None:
    if len(raw) < TELEM_SIZE:
        return None
    vals = struct.unpack(TELEM_FMT, raw[:TELEM_SIZE])
    if vals[0] != TELEM_MAGIC:
        return None
    keys = (
        "magic",
        "yaw",
        "pitch",
        "roll",
        "ax",
        "ay",
        "az",
        "gx",
        "gy",
        "gz",
        "dt",
        "healthy",
        "ready",
    )
    return dict(zip(keys, vals))


class ProbeReader(threading.Thread):
    def __init__(self, chip: str, addr: int, hz: float) -> None:
        super().__init__(daemon=True, name="stlink-reader")
        self.chip = chip
        self.addr = addr
        self.period = 1.0 / max(hz, 1.0)
        self.lock = threading.Lock()
        self.telem: dict | None = None
        self.status = "connecting"
        self.age = 0.0
        self.stop_evt = threading.Event()
        self.session = None

    def snapshot(self) -> tuple[dict | None, str, float]:
        with self.lock:
            return self.telem, self.status, self.age

    def _set(self, telem: dict | None, status: str) -> None:
        with self.lock:
            if telem is not None:
                self.telem = telem
                self.age = 0.0
            self.status = status

    def run(self) -> None:
        from pyocd.core.helpers import ConnectHelper

        try:
            self.session = ConnectHelper.session_with_chosen_probe(
                target_override=self.chip,
                connect_mode="attach",
                options={"frequency": 400000, "auto_unlock": False},
            )
            if self.session is None:
                self._set(None, "no ST-Link")
                return
            self.session.open()
            self._set(None, "attached")
        except Exception as exc:
            self._set(None, f"attach fail: {exc}")
            return

        target = self.session.target
        word_count = (TELEM_SIZE + 3) // 4
        while not self.stop_evt.is_set():
            t0 = time.monotonic()
            try:
                words = target.read_memory_block32(self.addr, word_count)
                raw = b"".join(int(w).to_bytes(4, "little") for w in words)[:TELEM_SIZE]
                telem = parse_telem(raw)
                self._set(telem, "live" if telem else "bad magic")
            except Exception as exc:
                self._set(None, f"read fail: {exc}")
            with self.lock:
                self.age += self.period
            leftover = self.period - (time.monotonic() - t0)
            if leftover > 0:
                self.stop_evt.wait(leftover)

        if self.session is not None:
            try:
                self.session.close()
            except Exception:
                pass

    def stop(self) -> None:
        self.stop_evt.set()


def main() -> int:
    parser = argparse.ArgumentParser(description="Chassis attitude 3D view via ST-Link")
    parser.add_argument("--chip", default="cortex_m")
    parser.add_argument("--addr", default="")
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument(
        "--map",
        default=str(Path(__file__).resolve().parents[1] / "Output" / "atk_f407.map"),
    )
    args = parser.parse_args()
    addr = int(args.addr, 0) if args.addr else find_telem_addr(Path(args.map))

    reader = ProbeReader(args.chip, addr, args.hz)
    reader.start()
    print(f"reader started, imu_telem @ 0x{addr:08X}")

    import matplotlib

    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    verts0, faces, colors = chassis_faces()
    fig = plt.figure("chassis attitude", figsize=(8.5, 6.5))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-1.5, 1.5)
    ax.set_xlabel("X fwd")
    ax.set_ylabel("Y left")
    ax.set_zlabel("Z up")
    collection = Poly3DCollection(
        [verts0[idx] for idx in faces],
        facecolors=colors,
        edgecolor="k",
        linewidths=0.6,
        alpha=0.92,
    )
    ax.add_collection3d(collection)
    hud = ax.text2D(0.02, 0.96, "", transform=ax.transAxes, family="monospace", va="top")
    ax.view_init(elev=22, azim=-60)

    def update(_frame):
        telem, status, age = reader.snapshot()
        if telem is None:
            hud.set_text(f"{status}\nwaiting for sample...")
            return collection, hud

        r = rot_zyx(telem["yaw"], telem["pitch"], telem["roll"])
        verts = verts0 @ r.T
        collection.set_verts([verts[idx] for idx in faces])
        stale = "" if status == "live" else f"  [{status}  stale {age:.1f}s]"
        hud.set_text(
            "yaw {:+6.1f}  pitch {:+6.1f}  roll {:+6.1f} deg{stale}\n"
            "acc {:+5.2f} {:+5.2f} {:+5.2f} g    |a|={:4.2f}\n"
            "gyr {:+6.1f} {:+6.1f} {:+6.1f} dps  dt={:.4f}s\n"
            "ready={} healthy={}".format(
                np.degrees(telem["yaw"]),
                np.degrees(telem["pitch"]),
                np.degrees(telem["roll"]),
                telem["ax"],
                telem["ay"],
                telem["az"],
                float(np.linalg.norm([telem["ax"], telem["ay"], telem["az"]])),
                telem["gx"],
                telem["gy"],
                telem["gz"],
                telem["dt"],
                telem["ready"],
                telem["healthy"],
                stale=stale,
            )
        )
        return collection, hud

    _anim = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        reader.stop()
        reader.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
