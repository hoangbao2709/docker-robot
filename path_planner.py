#!/usr/bin/env python3
"""
path_planner.py
----------------
Planner A* đơn giản, an toàn cho OccupancyGrid:

- Làm việc trên grid nhị phân: 0 = free (trắng + xám), 1 = obstacle (tường đen).
- Có inflate vật cản theo safe_clearance_m, nhưng chỉ dùng mỏng, không chặn plan vô cớ:
    + Nếu start/goal nằm trong vùng cấm (do inflate hoặc do discretization),
      sẽ tự động dời sang ô free gần nhất.
- A* 8-neighborhood, chống corner-cutting.
- Không dùng cost-map và smoothing để tránh đường vòng kỳ cục.
"""

import math
import heapq
from collections import deque
from typing import List, Optional, Tuple

import numpy as np
from nav_msgs.msg import MapMetaData


# ==============================================================
#                 world <-> cell helper
# ==============================================================

def world_to_cell(x: float, y: float, info: MapMetaData) -> Optional[Tuple[int, int]]:
    """Chuyển (x, y) world -> (r, c) cell, hoặc None nếu ngoài map."""
    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y

    c = int((x - ox) / res)
    r = int((y - oy) / res)

    if r < 0 or c < 0 or r >= info.height or c >= info.width:
        return None
    return r, c


def cell_to_world(r: int, c: int, info: MapMetaData) -> Tuple[float, float]:
    """Chuyển (r, c) cell -> tâm cell (x, y) trong frame map."""
    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y

    x = ox + (c + 0.5) * res
    y = oy + (r + 0.5) * res
    return x, y


# ==============================================================
#                    Inflate vật cản (mỏng)
# ==============================================================

def inflate_grid(grid: np.ndarray, res_m: float, clearance_m: float) -> np.ndarray:
    """
    Nở vật cản theo bán kính clearance_m (m).
    grid: 0 = free, 1 = obstacle (tường).
    """
    if clearance_m <= 0.0:
        return grid.copy()

    radius_cells = int(math.ceil(clearance_m / max(res_m, 1e-6)))
    if radius_cells <= 0:
        return grid.copy()

    h, w = grid.shape
    inflated = grid.copy()

    obstacle_indices = np.argwhere(grid != 0)
    if obstacle_indices.size == 0:
        return inflated

    offsets: List[Tuple[int, int]] = []
    r2 = radius_cells * radius_cells
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dr * dr + dc * dc <= r2:
                offsets.append((dr, dc))

    for r0, c0 in obstacle_indices:
        for dr, dc in offsets:
            r = r0 + dr
            c = c0 + dc
            if 0 <= r < h and 0 <= c < w:
                inflated[r, c] = 1

    return inflated


# ==============================================================
#   Tìm ô free gần nhất (dùng cho cả start & goal bị dính tường)
# ==============================================================

def find_nearest_free_cell(
    grid: np.ndarray,
    cell: Tuple[int, int],
    max_radius: int = 30,
) -> Optional[Tuple[int, int]]:
    """
    Tìm ô free gần nhất xung quanh cell trong grid 0/1.
    Dùng khi start/goal rơi vào obstacle do inflate hoặc discretization.
    """
    h, w = grid.shape
    cr, cc = cell

    if not (0 <= cr < h and 0 <= cc < w):
        return None

    if grid[cr, cc] == 0:
        return cell

    visited = np.zeros_like(grid, dtype=bool)
    q = deque()
    q.append((cr, cc, 0))
    visited[cr, cc] = True

    while q:
        r, c, d = q.popleft()
        if d > max_radius:
            break

        if grid[r, c] == 0:
            return (r, c)

        for dr, dc in [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and not visited[nr, nc]:
                visited[nr, nc] = True
                q.append((nr, nc, d + 1))

    return None


# ==============================================================
#                          A*
# ==============================================================

def a_star_grid(
    grid: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> Optional[List[Tuple[int, int]]]:
    """
    A* 8-neighborhood trên grid:
      grid: 0 = free, 1 = obstacle.
      start, goal: (r, c) đã đảm bảo nằm trong ô free.
    """
    h, w = grid.shape

    def in_bounds(r: int, c: int) -> bool:
        return 0 <= r < h and 0 <= c < w

    def is_free(r: int, c: int) -> bool:
        return grid[r, c] == 0

    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    neighbors = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1),
    ]

    open_heap: List[Tuple[float, float, Tuple[int, int]]] = []
    g_score = {start: 0.0}
    came_from: dict[Tuple[int, int], Tuple[int, int]] = {}

    heapq.heappush(open_heap, (heuristic(start, goal), 0.0, start))

    while open_heap:
        f, g_curr, current = heapq.heappop(open_heap)

        if current == goal:
            # reconstruct path
            path_cells = [current]
            while current in came_from:
                current = came_from[current]
                path_cells.append(current)
            path_cells.reverse()
            return path_cells

        if g_curr > g_score.get(current, float("inf")):
            continue

        cr, cc = current

        for dr, dc in neighbors:
            nr, nc = cr + dr, cc + dc

            if not in_bounds(nr, nc) or not is_free(nr, nc):
                continue

            # Chống corner-cutting: nếu đi chéo thì 2 ô cạnh phải free
            if dr != 0 and dc != 0:
                if not (is_free(cr + dr, cc) and is_free(cr, cc + dc)):
                    continue

            step_cost = math.hypot(dr, dc)
            tentative_g = g_curr + step_cost

            if tentative_g < g_score.get((nr, nc), float("inf")):
                g_score[(nr, nc)] = tentative_g
                came_from[(nr, nc)] = current
                f_new = tentative_g + heuristic((nr, nc), goal)
                heapq.heappush(open_heap, (f_new, tentative_g, (nr, nc)))

    return None


def _bresenham_cells(a: Tuple[int, int], b: Tuple[int, int]) -> List[Tuple[int, int]]:
    """Return the grid cells crossed by the segment from a to b."""
    r0, c0 = a
    r1, c1 = b

    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1

    cells: List[Tuple[int, int]] = []
    r, c = r0, c0

    if dc > dr:
        err = dc // 2
        while c != c1:
            cells.append((r, c))
            err -= dr
            if err < 0:
                r += sr
                err += dc
            c += sc
    else:
        err = dr // 2
        while r != r1:
            cells.append((r, c))
            err -= dc
            if err < 0:
                c += sc
                err += dr
            r += sr

    cells.append((r1, c1))
    return cells


def has_line_of_sight(
    grid: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> bool:
    """Check whether a straight segment stays inside free cells."""
    h, w = grid.shape
    for r, c in _bresenham_cells(start, goal):
        if not (0 <= r < h and 0 <= c < w):
            return False
        if grid[r, c] != 0:
            return False
    return True


def simplify_cell_path(
    grid: np.ndarray,
    cell_path: List[Tuple[int, int]],
) -> List[Tuple[int, int]]:
    """
    Remove intermediate A* waypoints when the current anchor can already see
    a farther waypoint on the inflated planning grid.
    """
    if len(cell_path) <= 2:
        return cell_path

    simplified = [cell_path[0]]
    anchor_idx = 0

    while anchor_idx < len(cell_path) - 1:
        furthest_idx = anchor_idx + 1
        probe_idx = furthest_idx

        while probe_idx < len(cell_path):
            if not has_line_of_sight(grid, cell_path[anchor_idx], cell_path[probe_idx]):
                break
            furthest_idx = probe_idx
            probe_idx += 1

        simplified.append(cell_path[furthest_idx])
        anchor_idx = furthest_idx

    return simplified


# ==============================================================
#                      API chính: plan_path
# ==============================================================

def plan_path(
    grid: np.ndarray,
    map_info: MapMetaData,
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    logger=None,
    safe_clearance_m: float = 0.05,
) -> Optional[List[Tuple[float, float]]]:
    """
    Tính đường đi từ start_xy -> goal_xy.

    - grid: 0 = free, 1 = obstacle (tường).
    - safe_clearance_m: bán kính inflate mỏng quanh tường.
      Nếu inflate làm bế tắc, planner vẫn cố snap start/goal ra vùng free.
    """
    log = logger.info if logger is not None else print
    warn = logger.warn if logger is not None else print

    if grid is None or grid.ndim != 2:
        warn("[plan_path] grid invalid")
        return None

    # World -> cell trên grid gốc
    start_cell = world_to_cell(start_xy[0], start_xy[1], map_info)
    goal_cell = world_to_cell(goal_xy[0], goal_xy[1], map_info)

    if start_cell is None or goal_cell is None:
        warn("[plan_path] start hoặc goal ngoài map")
        return None

    res = float(map_info.resolution)
    if res <= 0.0:
        warn("[plan_path] resolution <= 0")
        return None

    # Inflate mỏng quanh tường (nếu cần)
    if safe_clearance_m > 0.0:
        grid_for_plan = inflate_grid(grid, res, safe_clearance_m)
        label = "inflated"
    else:
        grid_for_plan = grid.copy()
        label = "raw"

    h, w = grid_for_plan.shape
    sr, sc = start_cell
    gr, gc = goal_cell

    # Snap start/goal nếu rơi vào obstacle
    new_start = find_nearest_free_cell(grid_for_plan, (sr, sc))
    if new_start is None:
        warn(f"[plan_path] {label}: start bị bao vây, không tìm được ô free.")
        return None
    if new_start != (sr, sc):
        log(f"[plan_path] {label}: dời start {start_cell} -> {new_start}")
    sr, sc = new_start

    new_goal = find_nearest_free_cell(grid_for_plan, (gr, gc))
    if new_goal is None:
        warn(f"[plan_path] {label}: goal bị bao vây, không tìm được ô free.")
        return None
    if new_goal != (gr, gc):
        log(f"[plan_path] {label}: dời goal {goal_cell} -> {new_goal}")
    gr, gc = new_goal

    log(
        f"[plan_path] {label}: grid {h}x{w}, "
        f"start_cell=({sr},{sc}), goal_cell=({gr},{gc})"
    )

    cell_path = a_star_grid(grid_for_plan, (sr, sc), (gr, gc))
    if cell_path is None:
        warn(f"[plan_path] {label}: A* không tìm được đường.")
        return None

    smooth_cell_path = simplify_cell_path(grid_for_plan, cell_path)

    # Cell -> world
    path_xy: List[Tuple[float, float]] = []
    for r, c in smooth_cell_path:
        x, y = cell_to_world(r, c, map_info)
        path_xy.append((x, y))

    log(
        f"[plan_path] {label}: Path length = {len(path_xy)} "
        f"(raw={len(cell_path)} cells, smooth={len(smooth_cell_path)} cells)"
    )
    return path_xy
