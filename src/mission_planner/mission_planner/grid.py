"""A* на сетке; unknown и пространство за картой считаются препятствием."""
from heapq import heappop, heappush
from math import ceil, floor, hypot, isfinite, sqrt


class PlanningError(ValueError):
    """Невозможно построить допустимый путь."""


def inflated_grid(width, height, resolution, data, radius, threshold=50):
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise PlanningError('Некорректный размер карты')
    if not isfinite(resolution) or resolution <= 0 or not isfinite(radius) or radius < 0:
        raise PlanningError('Некорректное разрешение или радиус')
    if not 1 <= threshold <= 100 or any(v < -1 or v > 100 for v in data):
        raise PlanningError('Некорректные значения occupancy')
    # По половине диагонали для препятствия и любой точки клетки маршрута.
    reach = radius + resolution * 2 ** .5
    if not isfinite(reach) or not isfinite(reach / resolution):
        raise PlanningError('Радиус слишком велик для разрешения карты')
    cell_reach = min(reach / resolution, hypot(width, height))
    cells = min(height - 1, floor(cell_reach))
    spans = [(dy, min(width - 1, floor(sqrt(max(0., cell_reach ** 2 - dy ** 2)))))
             for dy in range(-cells, cells + 1)]
    # Разностные строки не обходят каждую клетку диска для каждого препятствия.
    rows = [[0] * (width + 1) for _ in range(height)]
    blocked = set()
    for y in range(height):
        for x in range(width):
            if min(x, y, width - x - 1, height - y - 1) * resolution < radius:
                blocked.add((x, y))
            value = data[y * width + x]
            if value < 0 or value >= threshold:
                for dy, half_span in spans:
                    py = y + dy
                    if 0 <= py < height:
                        rows[py][max(0, x - half_span)] += 1
                        rows[py][min(width, x + half_span + 1)] -= 1
    for y, row in enumerate(rows):
        count = 0
        for x in range(width):
            count += row[x]
            if count:
                blocked.add((x, y))
    return blocked


def astar(width, height, blocked, start, goal):
    for name, point in [('Старт', start), ('Цель', goal)]:
        if not (0 <= point[0] < width and 0 <= point[1] < height):
            raise PlanningError(f'{name} за границами карты')
        if point in blocked:
            raise PlanningError(f'{name} в препятствии или зоне inflation')
    queue = [(hypot(goal[0] - start[0], goal[1] - start[1]), 0., start)]
    costs, parents = {start: 0.}, {}
    while queue:
        _, cost, point = heappop(queue)
        if cost > costs[point]:
            continue
        if point == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parents[path[-1]])
            return path[::-1]
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            nxt = point[0] + dx, point[1] + dy
            if not (0 <= nxt[0] < width and 0 <= nxt[1] < height) or nxt in blocked:
                continue
            if dx and dy and ((point[0] + dx, point[1]) in blocked
                              or (point[0], point[1] + dy) in blocked):
                continue
            candidate = cost + hypot(dx, dy)
            if candidate < costs.get(nxt, float('inf')):
                parents[nxt], costs[nxt] = point, candidate
                estimate = candidate + hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heappush(queue, (estimate, candidate, nxt))
    raise PlanningError('Цель недостижима')


def segment_cells(a, b):
    """Клетки, через которые проходит отрезок a→b; координаты в клетках.

    Консервативно: в точках пересечения линий сетки, включая узлы, берутся
    все соседние клетки, поэтому касание угла занятой клетки тоже считается.
    """
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    params = {0., 1.}
    for start, delta in ((ax, dx), (ay, dy)):
        if delta:
            low, high = sorted((start, start + delta))
            params.update((line - start) / delta for line in range(floor(low) + 1, ceil(high)))
    ts = sorted(params)
    cells = {(floor(ax + (t0 + t1) / 2 * dx), floor(ay + (t0 + t1) / 2 * dy))
             for t0, t1 in zip(ts, ts[1:])}
    for t in ts:
        x, y = ax + t * dx, ay + t * dy
        cells.update((floor(x + ox), floor(y + oy)) for ox in (-1e-9, 1e-9) for oy in (-1e-9, 1e-9))
    return cells


def shortcut_path(points, width, height, blocked):
    """Убирает лестницу A*: точка остаётся, только если без неё отрезок задел бы
    занятую клетку или край карты. Соседние точки входа должны быть видимы."""
    def visible(a, b):
        return all(0 <= x < width and 0 <= y < height and (x, y) not in blocked
                   for x, y in segment_cells(a, b))
    result, anchor = [points[0]], 0
    for k in range(2, len(points)):
        if not visible(points[anchor], points[k]):
            anchor = k - 1
            result.append(points[anchor])
    if len(points) > 1:
        result.append(points[-1])
    return result
