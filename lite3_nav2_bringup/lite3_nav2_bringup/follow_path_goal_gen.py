#!/usr/bin/env python3
"""
Generate a nav2 FollowPath goal YAML with orientations aligned to the path direction.

Example:
  ros2 run lite3_nav2_bringup follow_path_goal_gen --frame odom --points "2,-2;5,-4;6,0"
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import Iterable, List, Sequence, Tuple


def _parse_points(spec: str) -> List[Tuple[float, float]]:
    """
    Parse points from:
      "x1,y1;x2,y2;..."  (semicolon-separated)
    """
    spec = spec.strip()
    if not spec:
        raise ValueError("empty points spec")

    pts: List[Tuple[float, float]] = []
    for chunk in spec.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [p.strip() for p in chunk.split(",")]
        if len(parts) != 2:
            raise ValueError(f"invalid point '{chunk}', expected 'x,y'")
        pts.append((float(parts[0]), float(parts[1])))

    if len(pts) < 1:
        raise ValueError("no valid points parsed")
    return pts


def _yaws_from_points(points_xy: Sequence[Tuple[float, float]]) -> List[float]:
    n = len(points_xy)
    if n == 1:
        return [0.0]

    yaws: List[float] = []
    for i in range(n - 1):
        x0, y0 = points_xy[i]
        x1, y1 = points_xy[i + 1]
        yaws.append(math.atan2(y1 - y0, x1 - x0))
    yaws.append(yaws[-1])  # last pose: keep same heading as previous segment
    return yaws


def _quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert planar yaw (rad) to quaternion (x,y,z,w) for rotation about Z axis.
    """
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _fmt_f(x: float) -> str:
    # Compact but stable formatting for YAML
    return f"{x:.6f}".rstrip("0").rstrip(".") if abs(x) >= 1e-9 else "0.0"


def _emit_goal_yaml(
    points_xy: Sequence[Tuple[float, float]],
    yaws: Sequence[float],
    frame_id: str,
    z: float,
    controller_id: str,
    goal_checker_id: str,
    out: Iterable[str] | None = None,
) -> str:
    lines: List[str] = []
    lines.append('path:')
    lines.append('  header:')
    lines.append(f'    frame_id: {frame_id}')
    lines.append('  poses:')
    for (x, y), yaw in zip(points_xy, yaws):
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        lines.append('  - header:')
        lines.append(f'      frame_id: {frame_id}')
        lines.append('    pose:')
        lines.append(f'      position: {{x: {_fmt_f(x)}, y: {_fmt_f(y)}, z: {_fmt_f(z)}}}')
        lines.append(
            '      orientation: '
            + '{'
            + f'x: {_fmt_f(qx)}, y: {_fmt_f(qy)}, z: {_fmt_f(qz)}, w: {_fmt_f(qw)}'
            + '}'
        )
    lines.append(f'controller_id: {controller_id}')
    lines.append(f'goal_checker_id: {goal_checker_id}')
    return "\n".join(lines) + "\n"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate a nav2 FollowPath goal YAML with orientations aligned to the path direction."
    )
    parser.add_argument(
        "--points",
        required=True,
        help='Points as "x1,y1;x2,y2;...". Example: "2,-2;5,-4;6,0"',
    )
    parser.add_argument("--frame", default="odom", help="Frame id for the Path and PoseStamped headers.")
    parser.add_argument("--z", type=float, default=0.0, help="Z value for all poses (default: 0.0).")
    parser.add_argument("--controller-id", default="FollowPath", help='FollowPath controller id (default: "FollowPath").')
    parser.add_argument(
        "--goal-checker-id",
        default="general_goal_checker",
        help='Goal checker id (default: "general_goal_checker").',
    )

    args = parser.parse_args(argv)

    try:
        pts = _parse_points(args.points)
        yaws = _yaws_from_points(pts)
    except Exception as e:
        print(f"[follow_path_goal_gen] error: {e}", file=sys.stderr)
        return 2

    sys.stdout.write(
        _emit_goal_yaml(
            points_xy=pts,
            yaws=yaws,
            frame_id=args.frame,
            z=args.z,
            controller_id=args.controller_id,
            goal_checker_id=args.goal_checker_id,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


