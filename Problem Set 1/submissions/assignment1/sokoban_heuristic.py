from collections import deque
from typing import Dict, FrozenSet, Optional, Set, Tuple

from sokoban import SokobanProblem, SokobanState, SokobanLayout
from mathutils import Direction, Point, manhattan_distance

# Estimate uses closest crate distance minus one to stay admissible
def weak_heuristic(problem: SokobanProblem, state: SokobanState):
    return min(manhattan_distance(state.player, crate) for crate in state.crates) - 1

#TODO: Import any modules and write any functions you want to use

def _precompute_layout_info(layout: SokobanLayout) -> Tuple[FrozenSet[Point], Dict[Point, Dict[Point, int]]]:
    # Collect walkable tiles to know where crates and players may stand safely
    walkable = layout.walkable
    if not walkable:
        return frozenset(), {}

    # Precompute single step and double step vectors for reverse pushes
    vectors = [direction.to_vector() for direction in Direction]
    double_vectors = [Point(vec.x * 2, vec.y * 2) for vec in vectors]

    # Prepare per goal distance tables plus a reachability set we will grow
    goal_distances: Dict[Point, Dict[Point, int]] = {}
    reachable: Set[Point] = set(layout.goals)

    for goal in layout.goals:
        # Run reverse BFS from goal so we know pull counts to every cell
        distance_map: Dict[Point, int] = {goal: 0}
        queue = deque([goal])
        while queue:
            current = queue.popleft()
            base_distance = distance_map[current]
            for vec, dbl in zip(vectors, double_vectors):
                # Evaluate reverse push by checking crate and player support squares
                previous = current - vec
                player_cell = current - dbl
                if previous in walkable and player_cell in walkable and previous not in distance_map:
                    distance_map[previous] = base_distance + 1
                    queue.append(previous)
        goal_distances[goal] = distance_map
        reachable.update(distance_map.keys())
    return goal_distances


def _layout_info(problem: SokobanProblem) -> Tuple[FrozenSet[Point], Dict[Point, Dict[Point, int]]]:
    # Retrieve or refresh cached layout specific data for reuse across states
    cache_key = "sokoban_layout_info"
    cache = problem.cache()
    cached = cache.get(cache_key)
    if not cached or cached["layout"] is not problem.layout:
        # Recompute deadlocks and pull distances when layout object changes
        distances = _precompute_layout_info(problem.layout)
        cached = {"layout": problem.layout, "distances": distances}
        cache[cache_key] = cached
    return cached["distances"]


def _hungarian_min_cost(cost_matrix: list[list[int]]) -> int:
    # Return zero immediately for empty cost matrix to avoid indexing work
    size = len(cost_matrix)
    if size == 0:
        return 0

    # Initialise Hungarian potentials for rows, columns, and match tracking arrays
    dimension = len(cost_matrix[0])
    u = [0] * (size + 1)
    v = [0] * (dimension + 1)
    p = [0] * (dimension + 1)
    way = [0] * (dimension + 1)

    for row in range(1, size + 1):
        # Begin augmenting path search for this row using dummy column as root
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

    # Build explicit assignment array from column matches to compute cost
    assignment = [0] * (size + 1)
    for col in range(1, dimension + 1):
        if p[col]:
            assignment[p[col]] = col

    # Accumulate final matching cost using recorded assignments
    total = 0
    for row in range(1, size + 1):
        total += cost_matrix[row - 1][assignment[row] - 1]
    return total


def _minimum_matching_distance(crates: FrozenSet[Point], goals: FrozenSet[Point],
                               goal_distances: Dict[Point, Dict[Point, int]]) -> float:
    # Handle empty crate set so heuristic stays zero in solved configurations
    if not crates:
        return 0.0

    # Prepare cost matrix structure mapping crates to all goals
    crate_list = list(crates)
    goal_list = list(goals)
    cost_matrix: list[list[int]] = []
    inf_cost = 10 ** 6

    for crate in crate_list:
        # Build one cost row per crate and flag whether any goal is reachable
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

def strong_heuristic(problem: SokobanProblem, state: SokobanState) -> float:
    # get goal distance tables for current problem
    goal_distances = _layout_info(problem)

    # Compute lower bound via Hungarian assignment over crate goal distances
    matching_distance = _minimum_matching_distance(state.crates, problem.layout.goals, goal_distances)
    return float(matching_distance)
