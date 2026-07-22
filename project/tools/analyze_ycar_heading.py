from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


SOURCES = [
    Path(r"e:\seekfree_assistant-master\seekfree_assistant-master\【软件】逐飞助手V1版\Pictures\2026_07_20_17_08_30_Image.bmp"),
    Path(r"e:\seekfree_assistant-master\seekfree_assistant-master\【软件】逐飞助手V1版\Pictures\2026_07_20_17_08_41_Image.bmp"),
]
OUTPUT = Path(__file__).resolve().parents[1] / "debug" / "ycar_heading_analysis.png"


def ycar_group(binary, merge_gap=6):
    height, width = binary.shape
    seen = np.zeros_like(binary, dtype=bool)
    components = []
    for y in range(height):
        for x in range(width):
            if not binary[y, x] or seen[y, x]:
                continue
            todo = deque([(x, y)])
            seen[y, x] = True
            points = []
            while todo:
                px, py = todo.popleft()
                points.append((px, py))
                for dx, dy in ((-1, -1), (0, -1), (1, -1), (-1, 0),
                               (1, 0), (-1, 1), (0, 1), (1, 1)):
                    nx, ny = px + dx, py + dy
                    if (0 <= nx < width and 0 <= ny < height and
                            binary[ny, nx] and not seen[ny, nx]):
                        seen[ny, nx] = True
                        todo.append((nx, ny))
            components.append(points)

    boxes = []
    for points in components:
        xs, ys = zip(*points)
        boxes.append((min(xs), min(ys), max(xs), max(ys)))

    best = []
    for seed, points in enumerate(components):
        if len(points) < 8:
            continue
        keep = {seed}
        changed = True
        while changed:
            changed = False
            for index, candidate in enumerate(components):
                if index in keep or len(candidate) < 8:
                    continue
                ax0, ay0, ax1, ay1 = boxes[index]
                for kept in keep:
                    bx0, by0, bx1, by1 = boxes[kept]
                    dx = ax0 - bx1 if ax0 > bx1 else bx0 - ax1 if bx0 > ax1 else 0
                    dy = ay0 - by1 if ay0 > by1 else by0 - ay1 if by0 > ay1 else 0
                    if dx <= merge_gap and dy <= merge_gap:
                        keep.add(index)
                        changed = True
                        break
        merged = [point for index in keep for point in components[index]]
        if len(merged) > len(best):
            best = merged
    return np.asarray(best, dtype=float)


def convex_hull(points):
    pts = sorted(set(map(tuple, points.astype(int))))
    if len(pts) <= 1:
        return np.asarray(pts, dtype=float)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for point in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1], dtype=float)


def current_heading(points):
    center = points.mean(axis=0)
    delta = points - center
    cov = delta.T @ delta / len(points)
    values, vectors = np.linalg.eigh(cov)
    e1 = vectors[:, np.argmax(values)]
    e2 = np.array([-e1[1], e1[0]])
    p1 = delta @ e1
    p2 = delta @ e2
    m2_1, m2_2 = np.sum(p1**2), np.sum(p2**2)
    m3_1, m3_2 = np.sum(p1**3), np.sum(p2**3)
    skew1 = abs(m3_1) / (m2_1 * np.sqrt(m2_1 / len(points)) + 1e-6)
    skew2 = abs(m3_2) / (m2_2 * np.sqrt(m2_2 / len(points)) + 1e-6)
    axis, moment = (e1, m3_1) if skew1 >= skew2 else (e2, m3_2)
    rough = -axis if moment >= 0 else axis
    projection = delta @ rough
    apex = points[projection >= projection.max() - 2].mean(axis=0)
    vectors_from_apex = points - apex
    forward = vectors_from_apex @ rough
    side = rough[0] * vectors_from_apex[:, 1] - rough[1] * vectors_from_apex[:, 0]
    valid = forward < -2
    left = points[np.argmax(np.where(valid & (side >= 0), np.sum(vectors_from_apex**2, axis=1), -1))]
    right = points[np.argmax(np.where(valid & (side < 0), np.sum(vectors_from_apex**2, axis=1), -1))]
    u1, u2 = left - apex, right - apex
    u1 /= np.linalg.norm(u1)
    u2 /= np.linalg.norm(u2)
    heading = -(u1 + u2)
    heading /= np.linalg.norm(heading)
    return center, apex, left, right, heading, (skew1, skew2)


def hull_heading(points):
    hull = convex_hull(points)
    distances = np.sum((hull[:, None, :] - hull[None, :, :])**2, axis=2)
    end_a_index, end_b_index = np.unravel_index(np.argmax(distances), distances.shape)
    end_a, end_b = hull[end_a_index], hull[end_b_index]
    baseline = end_b - end_a
    baseline_norm = np.linalg.norm(baseline)
    signed_distance = np.cross(baseline, hull - end_a) / baseline_norm
    apex = hull[np.argmax(np.abs(signed_distance))]
    heading = apex - (end_a + end_b) * 0.5
    heading /= np.linalg.norm(heading)
    center = points.mean(axis=0)
    return center, apex, end_a, end_b, heading


def triangle_heading(points):
    hull = convex_hull(points)
    best_area = -1.0
    tips = None
    for i in range(len(hull) - 2):
        for j in range(i + 1, len(hull) - 1):
            for k in range(j + 1, len(hull)):
                area = abs(np.cross(hull[j] - hull[i], hull[k] - hull[i]))
                if area > best_area:
                    best_area = area
                    tips = [hull[i], hull[j], hull[k]]

    branch_scores = []
    for tip in tips:
        delta = points - tip
        distance2 = np.sum(delta**2, axis=1)
        local = delta[(distance2 >= 2.0) & (distance2 <= 12.0**2)]
        if len(local) < 3:
            branch_scores.append(-1.0)
            continue
        covariance = local.T @ local / len(local)
        values = np.linalg.eigvalsh(covariance)
        branch_scores.append(values[0] / (values[1] + 1e-6))

    apex_index = int(np.argmax(branch_scores))
    apex = tips[apex_index]
    ends = [tips[index] for index in range(3) if index != apex_index]
    heading = apex - (ends[0] + ends[1]) * 0.5
    heading /= np.linalg.norm(heading)
    center = points.mean(axis=0)
    return center, apex, ends[0], ends[1], heading, branch_scores


def extrema_triangle_heading(points):
    angles = np.arange(16) * (2.0 * np.pi / 16.0)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    indices = np.argmax(points @ directions.T, axis=0)
    candidates = np.unique(points[indices], axis=0)
    best_area = -1.0
    tips = None
    for i in range(len(candidates) - 2):
        for j in range(i + 1, len(candidates) - 1):
            for k in range(j + 1, len(candidates)):
                a = candidates[j] - candidates[i]
                b = candidates[k] - candidates[i]
                area = abs(a[0] * b[1] - a[1] * b[0])
                if area > best_area:
                    best_area = area
                    tips = [candidates[i], candidates[j], candidates[k]]
    branch_scores = []
    for tip in tips:
        delta = points - tip
        distance2 = np.sum(delta**2, axis=1)
        local = delta[(distance2 >= 2.0) & (distance2 <= 12.0**2)]
        covariance = local.T @ local / max(len(local), 1)
        values = np.linalg.eigvalsh(covariance)
        branch_scores.append(values[0] / (values[1] + 1e-6))
    apex_index = int(np.argmax(branch_scores))
    apex = tips[apex_index]
    ends = [tips[index] for index in range(3) if index != apex_index]
    heading = apex - (ends[0] + ends[1]) * 0.5
    heading /= np.linalg.norm(heading)
    return points.mean(axis=0), apex, ends[0], ends[1], heading, branch_scores


scale = 6
panels = []
for source in SOURCES:
    gray = np.asarray(Image.open(source).convert("L"))
    points = ycar_group(gray > 200)
    current = current_heading(points)
    hull = hull_heading(points)
    triangle = triangle_heading(points)
    extrema = extrema_triangle_heading(points)
    panels.append((source.name, gray, current, hull, triangle, extrema, len(points)))

panel_width = panels[0][1].shape[1] * scale
panel_height = panels[0][1].shape[0] * scale
canvas = Image.new("RGB", (panel_width * 4, panel_height * len(panels)), "black")
draw = ImageDraw.Draw(canvas)

for row, (name, gray, current, hull, triangle, extrema, count) in enumerate(panels):
    base = Image.fromarray(gray).resize((panel_width, panel_height), Image.Resampling.NEAREST).convert("RGB")
    for col, result in enumerate((current, hull, triangle, extrema)):
        x_offset = col * panel_width
        y_offset = row * panel_height
        canvas.paste(base, (x_offset, y_offset))
        center, apex, left, right, heading = result[:5]
        point = lambda p: (x_offset + int(p[0] * scale), y_offset + int(p[1] * scale))
        draw.line([point(apex), point(left)], fill="lime", width=2)
        draw.line([point(apex), point(right)], fill="lime", width=2)
        draw.ellipse([point(apex - 1), point(apex + 1)], outline="red", width=2)
        end = center + heading * 24
        draw.line([point(center), point(end)], fill="yellow", width=3)
        draw.text((x_offset + 5, y_offset + 5),
                  f"{('current', 'hull', 'triangle', 'extrema16')[col]} n={count}", fill="cyan")
    print(name, "current_heading", current[4], "skew", current[5],
          "hull_heading", hull[4], "triangle_heading", triangle[4],
          "branch_scores", triangle[5], "extrema_heading", extrema[4],
          "extrema_scores", extrema[5])

OUTPUT.parent.mkdir(parents=True, exist_ok=True)
canvas.save(OUTPUT)
print("output", OUTPUT)
