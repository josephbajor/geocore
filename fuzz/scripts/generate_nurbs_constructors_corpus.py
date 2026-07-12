#!/usr/bin/env python3
"""Generate deterministic curve/surface NURBS constructor seeds."""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = ROOT / "fuzz" / "corpus" / "nurbs_constructors"


def encode(
    selector: int,
    degree_u: int,
    degree_v: int,
    knots_u: list[float],
    knots_v: list[float],
    points: list[tuple[float, float, float]],
    weights: list[float],
    parameters: tuple[float, float] = (0.3, 0.7),
    projection_point: tuple[float, float, float] = (0.25, 0.5, 1.0),
) -> bytes:
    header = bytes(
        [
            selector,
            degree_u,
            degree_v,
            len(knots_u),
            len(knots_v),
            len(points),
            len(weights),
        ]
    )
    values = [*parameters, *projection_point, *knots_u, *knots_v]
    values.extend(coordinate for point in points for coordinate in point)
    values.extend(weights)
    return header + b"".join(struct.pack("<d", value) for value in values)


def corpus_entries() -> dict[str, bytes]:
    line_knots = [0.0, 0.0, 1.0, 1.0]
    quadratic_knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
    curve_points = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0)]
    grid_2 = [
        (0.0, 0.0, 0.0),
        (0.0, 1.0, 0.25),
        (1.0, 0.0, -0.25),
        (1.0, 1.0, 0.0),
    ]
    grid_3 = [
        (float(u), float(v), 0.125 * float((u + 2 * v) % 3))
        for u in range(3)
        for v in range(3)
    ]
    rational_selector = 1 << 1
    projection_selector = 1 << 2
    surface_selector = 1 << 0
    split_v_selector = 1 << 3
    second_order_selector = 2 << 4
    return {
        "curve-polynomial-linear.nurbsseed": encode(
            projection_selector | (1 << 4),
            1,
            0,
            line_knots,
            [],
            [(0.0, 0.0, 0.0), (1.0, 2.0, 0.5)],
            [],
        ),
        "curve-rational-quadratic.nurbsseed": encode(
            rational_selector | projection_selector | second_order_selector,
            2,
            0,
            quadratic_knots,
            [],
            curve_points,
            [1.0, math.sqrt(0.5), 1.0],
        ),
        "curve-invalid-degree-zero.nurbsseed": encode(
            0, 0, 0, [0.0, 1.0], [], [(0.0, 0.0, 0.0)], []
        ),
        "curve-invalid-nonfinite-point.nurbsseed": encode(
            0,
            1,
            0,
            line_knots,
            [],
            [(0.0, 0.0, 0.0), (math.nan, 1.0, 0.0)],
            [],
        ),
        "curve-invalid-weight-count.nurbsseed": encode(
            rational_selector,
            1,
            0,
            line_knots,
            [],
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
            [1.0],
        ),
        "surface-polynomial-bilinear.nurbsseed": encode(
            surface_selector | projection_selector | second_order_selector,
            1,
            1,
            line_knots,
            line_knots,
            grid_2,
            [],
        ),
        "surface-rational-quadratic.nurbsseed": encode(
            surface_selector | rational_selector | split_v_selector | second_order_selector,
            2,
            2,
            quadratic_knots,
            quadratic_knots,
            grid_3,
            [1.0, 0.75, 1.25, 1.0, 1.5, 0.875, 1.0, 1.125, 1.0],
        ),
        "surface-invalid-point-count.nurbsseed": encode(
            surface_selector,
            1,
            1,
            line_knots,
            line_knots,
            grid_2[:-1],
            [],
        ),
        "surface-invalid-nonfinite-weight.nurbsseed": encode(
            surface_selector | rational_selector,
            1,
            1,
            line_knots,
            line_knots,
            grid_2,
            [1.0, math.nan, 1.0, 1.0],
        ),
    }


def write_corpus(output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    expected = corpus_entries()
    for stale in output.glob("*.nurbsseed"):
        if stale.name not in expected:
            stale.unlink()
    for name, data in expected.items():
        (output / name).write_bytes(data)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    write_corpus(args.output)


if __name__ == "__main__":
    main()
