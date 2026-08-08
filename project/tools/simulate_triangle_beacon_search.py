"""Compare beacon-search paths for a 3.5 m equilateral placement region.

The simulation starts at the user's example previous beacon position
(300, 175) cm and treats the flight to the configured search datum
(175, 0) cm as useful sensing, not dead travel.

No third-party packages are required. Results are written as CSV and SVG.
"""

from __future__ import annotations

import csv
import math
import random
from pathlib import Path


SIDE_CM = 350.0
START = (300.0, 175.0)
CENTER = (175.0, 0.0)
CONTINUE_POINT = (175.0, -175.0)
LANE_SPACING_CM = 175.0
SPIRAL_RADIAL_SPACING_CM = 110.0
DETECTION_RADII_CM = (100.0, 120.0, 140.0)
RANDOM_TARGETS = 20_000
RANDOM_SEED = 20260725
STEP_CM = 4.0

OUTPUT_DIR = Path(__file__).resolve().parent / "triangle_search_results"


def add(a, b):
    return a[0] + b[0], a[1] + b[1]


def sub(a, b):
    return a[0] - b[0], a[1] - b[1]


def scale(v, factor):
    return v[0] * factor, v[1] * factor


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def rotate(v, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return v[0] * c - v[1] * s, v[0] * s + v[1] * c


def triangle_vertices():
    """Center the triangle at CENTER; orient one vertex toward START."""
    circumradius = SIDE_CM / math.sqrt(3.0)
    start_direction = sub(START, CENTER)
    start_angle = math.atan2(start_direction[1], start_direction[0])
    return [
        add(CENTER, rotate((circumradius, 0.0), start_angle + i * 2.0 * math.pi / 3.0))
        for i in range(3)
    ]


VERTICES = triangle_vertices()


def sample_triangle(count):
    rng = random.Random(RANDOM_SEED)
    a, b, c = VERTICES
    points = []
    for _ in range(count):
        u = rng.random()
        v = rng.random()
        if u + v > 1.0:
            u = 1.0 - u
            v = 1.0 - v
        points.append(
            (
                a[0] + u * (b[0] - a[0]) + v * (c[0] - a[0]),
                a[1] + u * (b[1] - a[1]) + v * (c[1] - a[1]),
            )
        )
    return points


def densify(points, step_cm=STEP_CM):
    dense = [points[0]]
    cumulative = [0.0]
    total = 0.0
    for start, end in zip(points, points[1:]):
        length = distance(start, end)
        steps = max(1, math.ceil(length / step_cm))
        for index in range(1, steps + 1):
            ratio = index / steps
            point = (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
            )
            total += distance(dense[-1], point)
            dense.append(point)
            cumulative.append(total)
    return dense, cumulative


def common_prefix():
    return [START, CENTER]


def spiral_points(max_radius=245.0):
    points = [CENTER]
    radial_gain = SPIRAL_RADIAL_SPACING_CM / (2.0 * math.pi)
    theta = 0.0
    while radial_gain * theta < max_radius:
        radius = radial_gain * theta
        points.append(
            (
                CENTER[0] + radius * math.cos(theta - math.pi / 2.0),
                CENTER[1] + radius * math.sin(theta - math.pi / 2.0),
            )
        )
        theta += 0.06
    return points


def path_spiral():
    return common_prefix() + spiral_points()[1:]


def path_continue_then_spiral():
    return common_prefix() + [CONTINUE_POINT, CENTER] + spiral_points()[1:]


def path_center_spokes():
    ordered = sorted(VERTICES, key=lambda point: math.atan2(point[1] - CENTER[1], point[0] - CENTER[0]))
    points = common_prefix()
    for vertex in ordered:
        points.extend((vertex, CENTER))
    return points


def path_triangle_perimeter():
    nearest = min(range(3), key=lambda index: distance(CENTER, VERTICES[index]))
    ordered = [VERTICES[(nearest + offset) % 3] for offset in range(4)]
    return common_prefix() + ordered


def horizontal_intersections(y):
    intersections = []
    for p1, p2 in zip(VERTICES, VERTICES[1:] + VERTICES[:1]):
        if (p1[1] <= y <= p2[1]) or (p2[1] <= y <= p1[1]):
            if abs(p2[1] - p1[1]) < 1e-9:
                intersections.extend((p1[0], p2[0]))
            else:
                ratio = (y - p1[1]) / (p2[1] - p1[1])
                intersections.append(p1[0] + ratio * (p2[0] - p1[0]))
    if len(intersections) < 2:
        return None
    return min(intersections), max(intersections)


def path_center_out_raster():
    min_y = min(point[1] for point in VERTICES)
    max_y = max(point[1] for point in VERTICES)
    levels = [CENTER[1]]
    offset = LANE_SPACING_CM
    while CENTER[1] + offset <= max_y or CENTER[1] - offset >= min_y:
        if CENTER[1] + offset <= max_y:
            levels.append(CENTER[1] + offset)
        if CENTER[1] - offset >= min_y:
            levels.append(CENTER[1] - offset)
        offset += LANE_SPACING_CM

    points = common_prefix()
    reverse = False
    for y in levels:
        bounds = horizontal_intersections(y)
        if bounds is None:
            continue
        left = (bounds[0], y)
        right = (bounds[1], y)
        if reverse:
            points.extend((right, left))
        else:
            points.extend((left, right))
        reverse = not reverse
    return points


def path_continue_then_raster():
    raster = path_center_out_raster()
    # Preserve the user's idea: the continuation leg is useful coverage.
    # Do not fly back to CENTER merely to begin the fallback raster.
    return common_prefix() + [CONTINUE_POINT] + raster[2:]


PATH_BUILDERS = {
    "spiral": path_spiral,
    "continue_then_spiral": path_continue_then_spiral,
    "center_spokes": path_center_spokes,
    "triangle_perimeter": path_triangle_perimeter,
    "center_out_raster": path_center_out_raster,
    "continue_then_raster": path_continue_then_raster,
}


def first_detection_distance(target, dense_path, cumulative, radius):
    radius_sq = radius * radius
    for point, travelled in zip(dense_path, cumulative):
        dx = point[0] - target[0]
        dy = point[1] - target[1]
        if dx * dx + dy * dy <= radius_sq:
            return travelled
    return None


def percentile(values, fraction):
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def evaluate_targets(targets, path_data, radius):
    distances = []
    misses = 0
    total_path = path_data[1][-1]
    for target in targets:
        found_at = first_detection_distance(target, path_data[0], path_data[1], radius)
        if found_at is None:
            misses += 1
            distances.append(total_path)
        else:
            distances.append(found_at)
    return {
        "mean_cm": sum(distances) / len(distances),
        "median_cm": percentile(distances, 0.50),
        "p90_cm": percentile(distances, 0.90),
        "worst_cm": max(distances),
        "miss_rate": misses / len(targets),
        "path_length_cm": total_path,
    }


def write_svg(paths):
    all_points = [point for path, _ in paths.values() for point in path] + VERTICES
    min_x = min(point[0] for point in all_points) - 40.0
    max_x = max(point[0] for point in all_points) + 40.0
    min_y = min(point[1] for point in all_points) - 40.0
    max_y = max(point[1] for point in all_points) + 40.0
    width = 1100
    height = 760

    def project(point):
        x = (point[0] - min_x) / (max_x - min_x) * width
        y = height - (point[1] - min_y) / (max_y - min_y) * height
        return x, y

    colors = {
        "spiral": "#2563eb",
        "continue_then_spiral": "#7c3aed",
        "center_spokes": "#dc2626",
        "triangle_perimeter": "#ea580c",
        "center_out_raster": "#059669",
        "continue_then_raster": "#0891b2",
    }
    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
    ]
    triangle = " ".join(f"{project(point)[0]:.1f},{project(point)[1]:.1f}" for point in VERTICES)
    elements.append(
        f'<polygon points="{triangle}" fill="#f3f4f6" stroke="#111827" stroke-width="3"/>'
    )
    for name, (path, _) in paths.items():
        polyline = " ".join(f"{project(point)[0]:.1f},{project(point)[1]:.1f}" for point in path)
        elements.append(
            f'<polyline points="{polyline}" fill="none" stroke="{colors[name]}" '
            f'stroke-width="2.5" opacity="0.78"/>'
        )
    for label, point, color in (
        ("START", START, "#111827"),
        ("CENTER", CENTER, "#16a34a"),
        ("CONTINUE", CONTINUE_POINT, "#9333ea"),
    ):
        x, y = project(point)
        elements.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="6" fill="{color}"/>')
        elements.append(
            f'<text x="{x + 9:.1f}" y="{y - 9:.1f}" font-size="16" fill="{color}">{label}</text>'
        )
    legend_y = 25
    for name, color in colors.items():
        elements.append(
            f'<line x1="20" y1="{legend_y}" x2="55" y2="{legend_y}" '
            f'stroke="{color}" stroke-width="4"/>'
        )
        elements.append(
            f'<text x="65" y="{legend_y + 5}" font-size="15" fill="#111827">{name}</text>'
        )
        legend_y += 24
    elements.append("</svg>")
    (OUTPUT_DIR / "triangle_search_paths.svg").write_text("\n".join(elements), encoding="utf-8")


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    random_targets = sample_triangle(RANDOM_TARGETS)
    vertex_targets = VERTICES[1:]
    paths = {name: densify(builder()) for name, builder in PATH_BUILDERS.items()}

    rows = []
    for radius in DETECTION_RADII_CM:
        for target_model, targets in (
            ("uniform_inside_triangle", random_targets),
            ("remaining_triangle_vertices", vertex_targets),
        ):
            for name, path_data in paths.items():
                metrics = evaluate_targets(targets, path_data, radius)
                rows.append(
                    {
                        "detection_radius_cm": radius,
                        "target_model": target_model,
                        "strategy": name,
                        **metrics,
                    }
                )

    with (OUTPUT_DIR / "triangle_search_metrics.csv").open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    write_svg(paths)

    print("Triangle vertices:", [tuple(round(value, 1) for value in point) for point in VERTICES])
    print("Common start and return:", START, "->", CENTER)
    print()
    for radius in DETECTION_RADII_CM:
        print(f"Detection radius {radius:.0f} cm")
        subset = [
            row for row in rows
            if row["detection_radius_cm"] == radius
            and row["target_model"] == "uniform_inside_triangle"
        ]
        for row in sorted(subset, key=lambda item: (item["miss_rate"], item["mean_cm"])):
            print(
                f"  {row['strategy']:24s} mean={row['mean_cm']:6.1f} "
                f"p90={row['p90_cm']:6.1f} worst={row['worst_cm']:6.1f} "
                f"miss={row['miss_rate'] * 100:5.1f}%"
            )
        print()

    print("Wrote:", OUTPUT_DIR / "triangle_search_metrics.csv")
    print("Wrote:", OUTPUT_DIR / "triangle_search_paths.svg")


if __name__ == "__main__":
    main()
