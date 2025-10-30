from collections import deque
from typing import Dict, FrozenSet, Optional, Set, Tuple

from sokoban import SokobanProblem, SokobanState, SokobanLayout
from mathutils import Direction, Point, manhattan_distance

# This heuristic returns the distance between the player and the nearest crate as an estimate for the path cost
# While it is consistent, it does a bad job at estimating the actual cost thus the search will explore a lot of nodes before finding a goal
def weak_heuristic(problem: SokobanProblem, state: SokobanState):
    return min(manhattan_distance(state.player, crate) for crate in state.crates) - 1

#TODO: Import any modules and write any functions you want to use

_deadlock_cache: Dict[SokobanLayout, Tuple[FrozenSet[Point], Dict[Point, Dict[Point, int]]]] = {}

def _double_vector(vector: Point) -> Point:
    return Point(vector.x * 2, vector.y * 2)


def _precompute_layout_info(layout: SokobanLayout) -> Tuple[FrozenSet[Point], Dict[Point, Dict[Point, int]]]:
    walkable = layout.walkable
    if not walkable:
        return frozenset(), {}

    vectors = [direction.to_vector() for direction in Direction]
    double_vectors = [_double_vector(vec) for vec in vectors]

    goal_distances: Dict[Point, Dict[Point, int]] = {}
    reachable: Set[Point] = set(layout.goals)

    for goal in layout.goals:
        distance_map: Dict[Point, int] = {goal: 0}
        queue = deque([goal])
        while queue:
            current = queue.popleft()
            base_distance = distance_map[current]
            for vec, dbl in zip(vectors, double_vectors):
                previous = current - vec
                player_cell = current - dbl
                if previous in walkable and player_cell in walkable and previous not in distance_map:
                    distance_map[previous] = base_distance + 1
                    queue.append(previous)
        goal_distances[goal] = distance_map
        reachable.update(distance_map.keys())

    deadlocks = frozenset(position for position in walkable if position not in reachable)
    return deadlocks, goal_distances


def _collect_corridor_cells(start: Point,
                            axis_dirs: Tuple[Direction, Direction],
                            perpendicular_dirs: Tuple[Direction, Direction],
                            layout: SokobanLayout) -> Optional[Set[Point]]:
    walkable = layout.walkable
    for perp in perpendicular_dirs:
        if start + perp.to_vector() in walkable:
            return None

    cells: Set[Point] = {start}
    for direction in axis_dirs:
        current = start
        step = direction.to_vector()
        while True:
            next_pos = current + step
            if next_pos not in walkable:
                break
            if any(next_pos + perp.to_vector() in walkable for perp in perpendicular_dirs):
                return None
            cells.add(next_pos)
            current = next_pos

    return cells


def _in_dead_end_corridor(crate: Point, layout: SokobanLayout) -> bool:
    horizontal_cells = _collect_corridor_cells(
        crate,
        (Direction.LEFT, Direction.RIGHT),
        (Direction.UP, Direction.DOWN),
        layout,
    )
    if horizontal_cells is not None and not any(cell in layout.goals for cell in horizontal_cells):
        return True

    vertical_cells = _collect_corridor_cells(
        crate,
        (Direction.UP, Direction.DOWN),
        (Direction.LEFT, Direction.RIGHT),
        layout,
    )
    if vertical_cells is not None and not any(cell in layout.goals for cell in vertical_cells):
        return True

    return False


def _forms_2x2_deadlock(crate: Point, crates: FrozenSet[Point], layout: SokobanLayout) -> bool:
    if crate in layout.goals:
        return False

    patterns = [
        (Point(0, 0), Point(1, 0), Point(0, 1), Point(1, 1)),
        (Point(0, 0), Point(-1, 0), Point(0, 1), Point(-1, 1)),
        (Point(0, 0), Point(1, 0), Point(0, -1), Point(1, -1)),
        (Point(0, 0), Point(-1, 0), Point(0, -1), Point(-1, -1)),
    ]

    for offsets in patterns:
        cells = [crate + offset for offset in offsets]
        if any(cell in layout.goals for cell in cells):
            continue
        blocked = True
        for cell in cells:
            if cell in crates:
                continue
            if cell not in layout.walkable:
                continue
            blocked = False
            break
        if blocked:
            return True

    return False


def _hungarian_min_cost(cost_matrix: list[list[int]]) -> int:
    size = len(cost_matrix)
    if size == 0:
        return 0

    dimension = len(cost_matrix[0])
    u = [0] * (size + 1)
    v = [0] * (dimension + 1)
    p = [0] * (dimension + 1)
    way = [0] * (dimension + 1)

    for row in range(1, size + 1):
        p[0] = row
        j0 = 0
        minv = [float('inf')] * (dimension + 1)
        used = [False] * (dimension + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = float('inf')
            j1 = 0
            for col in range(1, dimension + 1):
                if used[col]:
                    continue
                cur = cost_matrix[i0 - 1][col - 1] - u[i0] - v[col]
                if cur < minv[col]:
                    minv[col] = cur
                    way[col] = j0
                if minv[col] < delta:
                    delta = minv[col]
                    j1 = col
            for col in range(dimension + 1):
                if used[col]:
                    u[p[col]] += delta
                    v[col] -= delta
                else:
                    minv[col] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break

    assignment = [0] * (size + 1)
    for col in range(1, dimension + 1):
        if p[col]:
            assignment[p[col]] = col

    total = 0
    for row in range(1, size + 1):
        total += cost_matrix[row - 1][assignment[row] - 1]
    return total


def _minimum_matching_distance(crates: FrozenSet[Point], goals: FrozenSet[Point], layout: SokobanLayout) -> float:
    if not crates:
        return 0.0

    goal_distances = get_goal_distance_maps(layout)
    crate_list = list(crates)
    goal_list = list(goals)
    cost_matrix: list[list[int]] = []
    inf_cost = 10 ** 6

    for crate in crate_list:
        row: list[int] = []
        reachable_goal = False
        for goal in goal_list:
            distance_map = goal_distances.get(goal, {})
            distance = distance_map.get(crate, inf_cost)
            if distance < inf_cost:
                reachable_goal = True
            row.append(distance)
        if not reachable_goal:
            return float('inf')
        cost_matrix.append(row)

    return float(_hungarian_min_cost(cost_matrix))


def get_deadlock_positions(layout: SokobanLayout) -> FrozenSet[Point]:
    """Cache static dead squares reachable by no crate."""
    if layout not in _deadlock_cache:
        _deadlock_cache[layout] = _precompute_layout_info(layout)
    return _deadlock_cache[layout][0]


def get_goal_distance_maps(layout: SokobanLayout) -> Dict[Point, Dict[Point, int]]:
    if layout not in _deadlock_cache:
        _deadlock_cache[layout] = _precompute_layout_info(layout)
    return _deadlock_cache[layout][1]


def strong_heuristic(problem: SokobanProblem, state: SokobanState) -> float:
    """
     Implements a strong heuristic for Sokoban using a combination of deadlock
     detection and the "Simple Lower Bound" method.

     1. Deadlock pruning: pre-computed static dead squares are combined with
         tunnel and 2x2 pattern checks inspired by the pruning ideas discussed
         at http://sokobano.de/wiki/index.php?title=Solver.

     2. Minimum matching lower bound: if no deadlocks are triggered, it
         computes the optimal bipartite assignment between crates and goals
         using precomputed push distances (walls only), following the
         "Minimum Matching Lower Bound" guideline on the same reference page.

     3. Player access bound: the final value also respects the Manhattan
         distance between the player and the closest crate, keeping the
         heuristic admissible while tightening the estimate.
    """
    
    # Pruning via domain-specific deadlock detection (static squares, tunnels, frozen blocks)
    deadlock_positions = get_deadlock_positions(problem.layout)
    for crate in state.crates:
        if crate in deadlock_positions:
            return float('inf')
        if _in_dead_end_corridor(crate, problem.layout):
            return float('inf')
        if _forms_2x2_deadlock(crate, state.crates, problem.layout):
            return float('inf')

    # Minimum matching lower bound on pushes (Hungarian algorithm over Manhattan costs)
    matching_distance = _minimum_matching_distance(state.crates, problem.layout.goals, problem.layout)
    if matching_distance == float('inf'):
        return float('inf')
    if matching_distance == 0:
        return 0.0

    player_to_crate = min(manhattan_distance(state.player, crate) for crate in state.crates)
    heuristic_value = max(matching_distance, player_to_crate)
    return float(heuristic_value)
  