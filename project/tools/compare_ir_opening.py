from collections import deque
from pathlib import Path
from time import perf_counter

import numpy as np
from PIL import Image, ImageDraw


SOURCE = Path(r"e:\seekfree_assistant-master\seekfree_assistant-master\【软件】逐飞助手V1版\Pictures\2026_07_20_14_59_49_Image.bmp")
OUTPUT = Path(__file__).resolve().parents[1] / "debug" / "vision_opening_comparison.png"
THRESHOLD = 220


def erode(binary: np.ndarray, square: bool) -> np.ndarray:
    padded = np.pad(binary, 1, constant_values=False)
    result = padded[1:-1, 1:-1].copy()
    offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if square:
        offsets += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
    height, width = binary.shape
    for dy, dx in offsets:
        result &= padded[1 + dy:1 + dy + height, 1 + dx:1 + dx + width]
    return result


def dilate(binary: np.ndarray, square: bool) -> np.ndarray:
    padded = np.pad(binary, 1, constant_values=False)
    result = padded[1:-1, 1:-1].copy()
    offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if square:
        offsets += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
    height, width = binary.shape
    for dy, dx in offsets:
        result |= padded[1 + dy:1 + dy + height, 1 + dx:1 + dx + width]
    return result


def opening(binary: np.ndarray, square: bool) -> np.ndarray:
    return dilate(erode(binary, square), square)


def components(binary: np.ndarray) -> list[tuple[int, int, int, int, int]]:
    height, width = binary.shape
    visited = np.zeros_like(binary, dtype=bool)
    found = []
    for y in range(height):
        for x in range(width):
            if not binary[y, x] or visited[y, x]:
                continue
            queue = deque([(x, y)])
            visited[y, x] = True
            points = []
            while queue:
                px, py = queue.popleft()
                points.append((px, py))
                for dx, dy in ((-1, -1), (0, -1), (1, -1), (-1, 0),
                               (1, 0), (-1, 1), (0, 1), (1, 1)):
                    nx, ny = px + dx, py + dy
                    if (0 <= nx < width and 0 <= ny < height and
                            binary[ny, nx] and not visited[ny, nx]):
                        visited[ny, nx] = True
                        queue.append((nx, ny))
            xs, ys = zip(*points)
            found.append((len(points), min(xs), min(ys), max(xs), max(ys)))
    return sorted(found, reverse=True)


gray = np.asarray(Image.open(SOURCE).convert("L"))
binary = gray >= THRESHOLD

start = perf_counter()
for _ in range(1000):
    cross_open = opening(binary, square=False)
cross_us = (perf_counter() - start) * 1_000_000 / 1000

start = perf_counter()
for _ in range(1000):
    square_open = opening(binary, square=True)
square_us = (perf_counter() - start) * 1_000_000 / 1000

panels = [
    ("Gray", gray),
    (f"Binary T={THRESHOLD}", binary * 255),
    ("Cross opening", cross_open * 255),
    ("3x3 opening", square_open * 255),
]
scale = 4
label_height = 24
panel_width = gray.shape[1] * scale
panel_height = gray.shape[0] * scale + label_height
canvas = Image.new("RGB", (panel_width * len(panels), panel_height), "white")
draw = ImageDraw.Draw(canvas)
for index, (label, data) in enumerate(panels):
    panel = Image.fromarray(data.astype(np.uint8), mode="L").resize(
        (panel_width, gray.shape[0] * scale), Image.Resampling.NEAREST)
    x = index * panel_width
    canvas.paste(panel.convert("RGB"), (x, label_height))
    draw.text((x + 6, 5), label, fill="black")

OUTPUT.parent.mkdir(parents=True, exist_ok=True)
canvas.save(OUTPUT)

print(f"source={SOURCE}")
print(f"range={gray.min()}..{gray.max()} threshold={THRESHOLD}")
print(f"binary_components={components(binary)[:10]}")
print(f"cross_components={components(cross_open)[:10]}")
print(f"square_components={components(square_open)[:10]}")
print(f"numpy_cross_us={cross_us:.1f} numpy_square_us={square_us:.1f}")
print(f"output={OUTPUT}")
