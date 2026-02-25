#!/usr/bin/env python3
"""
Drive the simulated delta robot's end effector through a square trajectory.

Uses the two-triangle method to convert Cartesian waypoints into shoulder joint
angles, then plays them back in the MuJoCo viewer.

Usage:
    uv run python scripts/square_trajectory.py
    uv run python scripts/square_trajectory.py --size 30 --z -250 --speed 0.5
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer


# ── Inverse kinematics────────


@dataclass
class DeltaKinematics:
    """Inverse kinematics solver for a symmetric delta robot."""

    upper_arm: float  # shoulder-to-elbow length (mm)
    lower_arm: float  # elbow-to-effector-joint length (mm)
    Fd: float  # base joint offset from centre (mm)
    Ed: float  # effector joint offset from centre (mm)

    def __post_init__(self) -> None:
        self._sqrt3 = math.sqrt(3.0)

    def inverse(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """
        Compute joint angles (degrees) for a target end-effector position.

        Reference frame: origin at base centre, Z axis points downward.
        """
        t1 = self._solve_arm(x, y, z)
        x2, y2 = self._rotate_120(x, y)
        t2 = self._solve_arm(x2, y2, z)
        x3, y3 = self._rotate_240(x, y)
        t3 = self._solve_arm(x3, y3, z)
        return (t1, t2, t3)

    def _solve_arm(self, x: float, y: float, z: float) -> float:
        L, l, Fd, Ed = self.upper_arm, self.lower_arm, self.Fd, self.Ed

        d = y - Ed + Fd
        W2 = z * z + d * d
        W = math.sqrt(W2) if W2 > 1e-12 else 1e-6
        alpha = math.degrees(math.asin(max(-1.0, min(1.0, d / W))))

        A2 = l * l - x * x
        cos_omega = (W2 + L * L - A2) / (2.0 * L * W)
        cos_omega = max(-1.0, min(1.0, cos_omega))
        omega = math.degrees(math.acos(cos_omega))

        return 90.0 + alpha - omega

    def _rotate_120(self, x: float, y: float) -> tuple[float, float]:
        s = self._sqrt3 / 2.0
        return (x * -0.5 - y * s, x * s + y * -0.5)

    def _rotate_240(self, x: float, y: float) -> tuple[float, float]:
        s = -self._sqrt3 / 2.0
        return (x * -0.5 - y * s, x * s + y * -0.5)


# ── Trajectory helpers ───────────────────────────────────────────────────────


def lerp_angles(
    a: tuple[float, ...], b: tuple[float, ...], t: float
) -> tuple[float, ...]:
    return tuple(a_i + (b_i - a_i) * t for a_i, b_i in zip(a, b))


def build_square_waypoints(
    dk: DeltaKinematics, half_size: float, z: float
) -> list[tuple[float, float, float]]:
    """Return shoulder angles (radians) for the 4 corners of a square."""
    corners_mm = [
        (-half_size, -half_size, z),
        (half_size, -half_size, z),
        (half_size, half_size, z),
        (-half_size, half_size, z),
    ]
    waypoints_rad = []
    for cx, cy, cz in corners_mm:
        t1, t2, t3 = dk.inverse(cx, cy, cz)
        waypoints_rad.append((math.radians(t1), math.radians(t2), math.radians(t3)))
    return waypoints_rad


# ── Main ─────────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="Delta robot square trajectory")
    parser.add_argument(
        "--size",
        type=float,
        default=80.0,
        help="Half-side length of the square in mm (default: 80)",
    )
    parser.add_argument(
        "--z",
        type=float,
        default=-250.0,
        help="Z height in mm, negative = below base (default: -250)",
    )
    parser.add_argument(
        "--speed", type=float, default=1.0, help="Seconds per edge (default: 1.0)"
    )
    parser.add_argument(
        "--loops",
        type=int,
        default=0,
        help="Number of loops, 0 = infinite (default: 0)",
    )
    args = parser.parse_args()

    # IK solver — same parameters as delta/python/delta_kinematics.py
    dk = DeltaKinematics(upper_arm=150.0, lower_arm=271.0, Fd=36.7, Ed=80.0)

    waypoints = build_square_waypoints(dk, args.size, args.z)
    n_wp = len(waypoints)
    edge_time = args.speed

    print(f"Square: ±{args.size}mm at z={args.z}mm, {edge_time}s per edge")
    for i, wp in enumerate(waypoints):
        degs = tuple(math.degrees(a) for a in wp)
        print(
            f"  Corner {i}: θ1={degs[0]:+.2f}°  θ2={degs[1]:+.2f}°  θ3={degs[2]:+.2f}°"
        )

    # Load MuJoCo model
    scene_path = (
        Path(__file__).parent.parent / "simulation" / "delta-robot" / "scene.xml"
    )

    plugin_dir = Path(mujoco.__file__).parent / "plugin"
    if plugin_dir.is_dir():
        for lib in plugin_dir.iterdir():
            if lib.suffix in (".dylib", ".so", ".dll"):
                mujoco.mj_loadPluginLibrary(str(lib))

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)

    shoulder_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in ("shoulder_1", "shoulder_2", "shoulder_3")
    ]

    # Use a control callback so the trajectory runs inside the viewer's own
    # simulation loop.  This avoids launch_passive (which needs mjpython on macOS).
    traj_state = {"start": None, "loop_count": 0}

    def controller(m, d):
        if traj_state["start"] is None:
            traj_state["start"] = d.time

        elapsed = d.time - traj_state["start"]
        total_loop_time = n_wp * edge_time

        if args.loops > 0 and elapsed >= args.loops * total_loop_time:
            wp = waypoints[0]
            for sid, angle in zip(shoulder_ids, wp):
                d.ctrl[sid] = angle
            return

        loop_elapsed = elapsed % total_loop_time
        edge_idx = int(loop_elapsed / edge_time)
        edge_frac = (loop_elapsed - edge_idx * edge_time) / edge_time

        wp_from = waypoints[edge_idx % n_wp]
        wp_to = waypoints[(edge_idx + 1) % n_wp]
        current = lerp_angles(wp_from, wp_to, edge_frac)

        for sid, angle in zip(shoulder_ids, current):
            d.ctrl[sid] = angle

        new_loop = int(elapsed / total_loop_time)
        if new_loop > traj_state["loop_count"]:
            traj_state["loop_count"] = new_loop
            print(f"  Loop {new_loop} complete")

    mujoco.set_mjcb_control(controller)

    print("\nLaunching viewer — close the window to stop.")
    print("  (press Space to pause, Backspace to reset)")
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
