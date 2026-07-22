from __future__ import annotations

import argparse
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw


EXPECTED_HEADINGS = {
    "2026_07_20_14_59_49_Image.bmp": (0.0, -1.0),
    "2026_07_20_15_30_32_Image.bmp": (-1.0, 0.0),
    "2026_07_20_15_31_21_Image.bmp": (-0.707, 0.707),
    "2026_07_20_15_32_13_Image.bmp": (-0.924, 0.383),
    "2026_07_20_15_38_47_Image.bmp": (-1.0, 0.0),
    "2026_07_20_17_08_30_Image.bmp": (0.0, -1.0),
    "2026_07_20_17_08_41_Image.bmp": (0.707, -0.707),
    "2026_07_21_11_31_58_Image.bmp": (0.0, -1.0),
    "2026_07_21_11_32_06_Image.bmp": (0.0, -1.0),
    "2026_07_21_11_32_18_Image.bmp": (0.0, -1.0),
    "2026_07_21_11_32_49_Image.bmp": (0.0, -1.0),
    "2026_07_21_11_33_05_Image.bmp": (0.383, -0.924),
    "2026_07_21_11_43_20_Image.bmp": (0.0, -1.0),
    "2026_07_21_11_43_32_Image.bmp": (0.0, -1.0),
    "2026_07_21_11_43_47_Image.bmp": (0.0, -1.0),
    "2026_07_21_12_13_12_Image.bmp": (0.555, -0.832),
}
EXPECTED_MISSES = {"2026_07_21_10_52_25_Image.bmp"}


@dataclass
class Detection:
    center: tuple[float, float]
    head: tuple[float, float]
    apex: tuple[float, float]
    ends: tuple[tuple[float, float], tuple[float, float]]
    score: float
    area: int
    arm_support: tuple[int, int]
    arm_bins: tuple[int, int]
    near_ratio: float
    off_ratio: float
    arm_lengths: tuple[float, float]
    opening_deg: float
    solidity: float
    enclosed_hole_pixels: int


def components(mask: list[list[bool]]) -> list[list[tuple[int, int]]]:
    height = len(mask)
    width = len(mask[0])
    seen = [[False] * width for _ in range(height)]
    result: list[list[tuple[int, int]]] = []
    for y in range(height):
        for x in range(width):
            if not mask[y][x] or seen[y][x]:
                continue
            queue = deque([(x, y)])
            seen[y][x] = True
            points: list[tuple[int, int]] = []
            while queue:
                px, py = queue.popleft()
                points.append((px, py))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        nx, ny = px + dx, py + dy
                        if (
                            0 <= nx < width
                            and 0 <= ny < height
                            and not seen[ny][nx]
                            and mask[ny][nx]
                        ):
                            seen[ny][nx] = True
                            queue.append((nx, ny))
            result.append(points)
    return result


def merge_nearby(
    groups: list[list[tuple[int, int]]], gap: int = 7
) -> list[list[tuple[int, int]]]:
    boxes = []
    for group in groups:
        xs = [p[0] for p in group]
        ys = [p[1] for p in group]
        boxes.append((min(xs), min(ys), max(xs), max(ys)))
    parent = list(range(len(groups)))

    def root(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for i in range(len(groups)):
        for j in range(i):
            ax0, ay0, ax1, ay1 = boxes[i]
            bx0, by0, bx1, by1 = boxes[j]
            dx = max(ax0 - bx1, bx0 - ax1, 0)
            dy = max(ay0 - by1, by0 - ay1, 0)
            if dx <= gap and dy <= gap:
                ri, rj = root(i), root(j)
                parent[ri] = rj

    merged: dict[int, list[tuple[int, int]]] = {}
    for index, group in enumerate(groups):
        merged.setdefault(root(index), []).extend(group)
    return list(merged.values())


def farthest(points: list[tuple[int, int]], origin: tuple[float, float]):
    return max(points, key=lambda p: (p[0] - origin[0]) ** 2 + (p[1] - origin[1]) ** 2)


def branch_score(points: list[tuple[int, int]], tip: tuple[int, int]) -> float:
    xx = yy = xy = 0.0
    count = 0
    for x, y in points:
        dx, dy = x - tip[0], y - tip[1]
        distance2 = dx * dx + dy * dy
        if 2 <= distance2 <= 196:
            xx += dx * dx
            yy += dy * dy
            xy += dx * dy
            count += 1
    if count < 3:
        return 0.0
    trace = xx + yy
    disc = math.sqrt((xx - yy) ** 2 + 4.0 * xy * xy)
    return (trace - disc) / (trace + disc + 1e-6)


def segment_distance(
    point: tuple[int, int],
    start: tuple[int, int],
    end: tuple[int, int],
) -> tuple[float, float]:
    vx, vy = end[0] - start[0], end[1] - start[1]
    length2 = vx * vx + vy * vy
    if length2 == 0:
        return math.dist(point, start), 0.0
    t = (
        (point[0] - start[0]) * vx + (point[1] - start[1]) * vy
    ) / length2
    t = max(0.0, min(1.0, t))
    qx, qy = start[0] + t * vx, start[1] + t * vy
    return math.hypot(point[0] - qx, point[1] - qy), t


def arm_metrics(
    points: list[tuple[int, int]],
    apex: tuple[int, int],
    ends: tuple[tuple[int, int], tuple[int, int]],
) -> tuple[tuple[int, int], tuple[int, int], float, float]:
    support = [0, 0]
    bins = [set(), set()]
    near = off = 0
    for point in points:
        distances = [
            segment_distance(point, apex, ends[0]),
            segment_distance(point, apex, ends[1]),
        ]
        arm = 0 if distances[0][0] <= distances[1][0] else 1
        distance, t = distances[arm]
        if distance <= 2.5:
            near += 1
            if t >= 0.08:
                support[arm] += 1
                bins[arm].add(min(7, int(t * 8.0)))
        if distance > 4.0:
            off += 1
    count = max(1, len(points))
    return (
        (support[0], support[1]),
        (len(bins[0]), len(bins[1])),
        near / count,
        off / count,
    )


def convex_hull(points: list[tuple[int, int]]) -> list[tuple[int, int]]:
    ordered = sorted(set(points))
    if len(ordered) <= 1:
        return ordered

    def cross(o, a, b):
        return (
            (a[0] - o[0]) * (b[1] - o[1])
            - (a[1] - o[1]) * (b[0] - o[0])
        )

    lower = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def polygon_area(points: list[tuple[int, int]]) -> float:
    return abs(
        sum(
            points[index][0] * points[(index + 1) % len(points)][1]
            - points[index][1] * points[(index + 1) % len(points)][0]
            for index in range(len(points))
        )
    ) * 0.5


def enclosed_hole_pixels(points: list[tuple[int, int]]) -> int:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x, max_x = min(xs) - 1, max(xs) + 1
    min_y, max_y = min(ys) - 1, max(ys) + 1
    foreground = set(points)
    outside = {(min_x, min_y)}
    queue = deque(outside)
    while queue:
        x, y = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (x + dx, y + dy)
            if (
                min_x <= neighbor[0] <= max_x
                and min_y <= neighbor[1] <= max_y
                and neighbor not in foreground
                and neighbor not in outside
            ):
                outside.add(neighbor)
                queue.append(neighbor)
    background_area = (max_x - min_x + 1) * (max_y - min_y + 1) - len(points)
    return background_area - len(outside)


def detect_group(points: list[tuple[int, int]]) -> Detection | None:
    if len(points) < 20:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    width = max(xs) - min(xs) + 1
    height = max(ys) - min(ys) + 1
    fill = len(points) / (width * height)
    if fill > 0.76 or max(width, height) / max(1, min(width, height)) > 4.5:
        return None

    center = (sum(xs) / len(xs), sum(ys) / len(ys))
    first = farthest(points, center)
    second = farthest(points, first)
    third = farthest(points, second)

    # Include directional extrema so thick/merged arms still offer apex candidates.
    candidates = {first, second, third}
    for angle_index in range(16):
        angle = angle_index * math.pi / 8.0
        ux, uy = math.cos(angle), math.sin(angle)
        candidates.add(max(points, key=lambda p: p[0] * ux + p[1] * uy))

    best = None
    candidate_list = list(candidates)
    for i in range(len(candidate_list)):
        for j in range(i + 1, len(candidate_list)):
            for k in range(j + 1, len(candidate_list)):
                triangle = (
                    candidate_list[i],
                    candidate_list[j],
                    candidate_list[k],
                )
                ax = triangle[1][0] - triangle[0][0]
                ay = triangle[1][1] - triangle[0][1]
                bx = triangle[2][0] - triangle[0][0]
                by = triangle[2][1] - triangle[0][1]
                triangle_area = abs(ax * by - ay * bx)
                if triangle_area < 12:
                    continue
                scores = [branch_score(points, tip) for tip in triangle]
                apex_index = max(range(3), key=scores.__getitem__)
                apex = triangle[apex_index]
                ends = [triangle[index] for index in range(3) if index != apex_index]
                len_a = math.dist(apex, ends[0])
                len_b = math.dist(apex, ends[1])
                if min(len_a, len_b) < 5.0:
                    continue
                symmetry = min(len_a, len_b) / max(len_a, len_b)
                apex_margin = scores[apex_index] - sorted(scores)[1]
                score = triangle_area * (0.5 + 0.5 * symmetry) * (1.0 + apex_margin)
                if best is None or score > best[0]:
                    best = (score, apex, ends[0], ends[1])

    if best is None:
        return None
    score, apex, end_a, end_b = best
    support, bins, near_ratio, off_ratio = arm_metrics(
        points, apex, (end_a, end_b)
    )
    arm_lengths = (math.dist(apex, end_a), math.dist(apex, end_b))
    arm_dot = (
        (end_a[0] - apex[0]) * (end_b[0] - apex[0])
        + (end_a[1] - apex[1]) * (end_b[1] - apex[1])
    ) / (arm_lengths[0] * arm_lengths[1])
    opening_deg = math.degrees(math.acos(max(-1.0, min(1.0, arm_dot))))
    hull_area = polygon_area(convex_hull(points))
    solidity = len(points) / max(1.0, hull_area)
    hole_pixels = enclosed_hole_pixels(points)
    if hole_pixels > 8:
        return None
    base_mid = ((end_a[0] + end_b[0]) * 0.5, (end_a[1] + end_b[1]) * 0.5)
    hx, hy = apex[0] - base_mid[0], apex[1] - base_mid[1]
    norm = math.hypot(hx, hy)
    if norm < 3.0:
        return None
    return Detection(
        center=center,
        head=(hx / norm, hy / norm),
        apex=apex,
        ends=(end_a, end_b),
        score=score,
        area=len(points),
        arm_support=support,
        arm_bins=bins,
        near_ratio=near_ratio,
        off_ratio=off_ratio,
        arm_lengths=arm_lengths,
        opening_deg=opening_deg,
        solidity=solidity,
        enclosed_hole_pixels=hole_pixels,
    )


def detect(path: Path, threshold: int) -> Detection | None:
    image = Image.open(path).convert("L")
    pixels = image.load()
    mask = [
        [pixels[x, y] >= threshold for x in range(image.width)]
        for y in range(image.height)
    ]
    groups = [group for group in components(mask) if len(group) >= 3]
    detections = [
        detection
        for group in merge_nearby(groups)
        if (detection := detect_group(group)) is not None
    ]
    return max(detections, key=lambda item: item.score, default=None)


def draw_result(path: Path, detection: Detection | None, output: Path) -> None:
    image = Image.open(path).convert("RGB")
    draw = ImageDraw.Draw(image)
    if detection:
        ax, ay = detection.apex
        for ex, ey in detection.ends:
            draw.line((ax, ay, ex, ey), fill=(0, 255, 0), width=1)
        cx, cy = detection.center
        hx, hy = detection.head
        draw.line((cx, cy, cx + hx * 20, cy + hy * 20), fill=(255, 0, 0), width=2)
        draw.ellipse((ax - 2, ay - 2, ax + 2, ay + 2), outline=(255, 255, 0))
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", nargs="+", type=Path)
    parser.add_argument("--threshold", type=int, default=220)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--contact-sheet", type=Path)
    parser.add_argument("--debug-components", action="store_true")
    args = parser.parse_args()
    failures = 0
    rendered: list[tuple[str, Image.Image]] = []
    for path in args.images:
        if args.debug_components:
            image = Image.open(path).convert("L")
            pixels = image.load()
            mask = [
                [pixels[x, y] >= args.threshold for x in range(image.width)]
                for y in range(image.height)
            ]
            stats = []
            point_component = {}
            for component_index, group in enumerate(components(mask)):
                if len(group) < 3:
                    continue
                for point in group:
                    point_component[point] = component_index
                xs = [point[0] for point in group]
                ys = [point[1] for point in group]
                width = max(xs) - min(xs) + 1
                height = max(ys) - min(ys) + 1
                stats.append(
                    (
                        len(group),
                        width,
                        height,
                        len(group) / (width * height),
                    )
                )
            print(f"PARTS {path.name} {sorted(stats, reverse=True)[:8]}")
        result = detect(path, args.threshold)
        if args.debug_components and result is not None:
            tips = (result.apex, result.ends[0], result.ends[1])
            print(
                "TIP_COMPONENTS",
                tuple(point_component.get(tip, -1) for tip in tips),
            )
        expected_heading = EXPECTED_HEADINGS.get(path.name)
        expected_miss = path.name in EXPECTED_MISSES
        if result is None:
            if not expected_miss:
                failures += 1
            print(f"MISS {path.name}")
        else:
            if expected_miss:
                failures += 1
                print(f"UNEXPECTED_DETECTION {path.name}")
            if expected_heading is not None:
                direction_dot = (
                    result.head[0] * expected_heading[0]
                    + result.head[1] * expected_heading[1]
                )
                if direction_dot < 0.8:
                    failures += 1
                    print(
                        f"WRONG_DIRECTION {path.name} dot={direction_dot:.3f}"
                    )
            print(
                f"OK {path.name} area={result.area} center=({result.center[0]:.1f},"
                f"{result.center[1]:.1f}) head=({result.head[0]:+.3f},"
                f"{result.head[1]:+.3f}) score={result.score:.1f} "
                f"support={result.arm_support} bins={result.arm_bins} "
                f"near={result.near_ratio:.2f} off={result.off_ratio:.2f} "
                f"arms=({result.arm_lengths[0]:.1f},"
                f"{result.arm_lengths[1]:.1f}) open={result.opening_deg:.1f} "
                f"solidity={result.solidity:.2f} "
                f"holes={result.enclosed_hole_pixels}"
            )
        if args.output_dir:
            output_path = args.output_dir / f"{path.stem}_detected.png"
            draw_result(path, result, output_path)
            rendered.append((path.name, Image.open(output_path).convert("RGB")))
    if args.contact_sheet and rendered:
        cell_width = max(image.width for _, image in rendered)
        cell_height = max(image.height for _, image in rendered) + 14
        columns = 3
        rows = (len(rendered) + columns - 1) // columns
        sheet = Image.new("RGB", (cell_width * columns, cell_height * rows), "black")
        draw = ImageDraw.Draw(sheet)
        for index, (name, image) in enumerate(rendered):
            x = (index % columns) * cell_width
            y = (index // columns) * cell_height
            sheet.paste(image, (x, y))
            draw.text((x + 2, y + image.height + 1), name[11:19], fill="white")
        args.contact_sheet.parent.mkdir(parents=True, exist_ok=True)
        sheet.save(args.contact_sheet)
    return failures


if __name__ == "__main__":
    raise SystemExit(main())
