from collections import deque  # import deque for breadth-first traversals
from typing import Dict, FrozenSet, Optional, Set, Tuple  # import typing helpers for type hints

from sokoban import SokobanProblem, SokobanState, SokobanLayout  # import Sokoban core types
from mathutils import Direction, Point, manhattan_distance  # import geometry utilities and distance helper

# This heuristic returns the distance between the player and the nearest crate as an estimate for the path cost
# While it is consistent, it does a bad job at estimating the actual cost thus the search will explore a lot of nodes before finding a goal
def weak_heuristic(problem: SokobanProblem, state: SokobanState):  # define baseline heuristic function
    return min(manhattan_distance(state.player, crate) for crate in state.crates) - 1  # compute naive player-to-crate distance minus one

#TODO: Import any modules and write any functions you want to use

def _precompute_layout_info(layout: SokobanLayout) -> Tuple[FrozenSet[Point], Dict[Point, Dict[Point, int]]]:  # build static layout caches
    walkable = layout.walkable  # grab every walkable square from the layout
    if not walkable:  # check if layout has no walkable squares
        return frozenset(), {}  # return empty deadlocks and empty distance map when board is trivial

    vectors = [direction.to_vector() for direction in Direction]  # generate movement vectors for each direction
    double_vectors = [Point(vec.x * 2, vec.y * 2) for vec in vectors]  # create doubled vectors to simulate reverse pushes

    goal_distances: Dict[Point, Dict[Point, int]] = {}  # allocate mapping from goal to its pull-distance table
    reachable: Set[Point] = set(layout.goals)  # seed reachable squares with all goal positions

    for goal in layout.goals:  # iterate through each goal square
        distance_map: Dict[Point, int] = {goal: 0}  # initialise distance map with the goal itself at zero cost
        queue = deque([goal])  # start BFS queue from the current goal
        while queue:  # process reverse-push BFS frontier
            current = queue.popleft()  # pop the next square to explore
            base_distance = distance_map[current]  # fetch distance already assigned to current square
            for vec, dbl in zip(vectors, double_vectors):  # examine each direction and corresponding doubled vector
                previous = current - vec  # compute predecessor crate position when pulling towards current
                player_cell = current - dbl  # compute player square required to execute the reverse move
                if previous in walkable and player_cell in walkable and previous not in distance_map:  # ensure both crate and player cells are valid and unseen
                    distance_map[previous] = base_distance + 1  # assign push count for predecessor position
                    queue.append(previous)  # enqueue predecessor to continue BFS
        goal_distances[goal] = distance_map  # store computed distance table for this goal
        reachable.update(distance_map.keys())  # add every discovered square to reachable set

    deadlocks = frozenset(position for position in walkable if position not in reachable)  # mark walkable squares unreachable from any goal as deadlocks
    return deadlocks, goal_distances  # return both the deadlock set and per-goal distances


def _layout_info(problem: SokobanProblem) -> Tuple[FrozenSet[Point], Dict[Point, Dict[Point, int]]]:  # retrieve cached layout data for given problem
    cache_key = "sokoban_layout_info"  # pick a constant cache key for storage
    cache = problem.cache()  # access reusable cache dictionary attached to problem
    cached = cache.get(cache_key)  # attempt to fetch cached layout metadata
    if not cached or cached["layout"] is not problem.layout:  # recompute if cache missing or layout changed
        deadlocks, distances = _precompute_layout_info(problem.layout)  # generate deadlocks and pull distances
        cached = {"layout": problem.layout, "deadlocks": deadlocks, "distances": distances}  # package newly computed data
        cache[cache_key] = cached  # store package back into cache for future calls
    return cached["deadlocks"], cached["distances"]  # return deadlock squares and distance tables


def _in_dead_end_corridor(crate: Point, layout: SokobanLayout) -> bool:  # detect tunnel-like dead-end positions
    walkable = layout.walkable  # access walkable squares from layout
    orientations = (  # define horizontal and vertical corridor configurations to test
        ((Direction.LEFT, Direction.RIGHT), (Direction.UP, Direction.DOWN)),  # first orientation: horizontal corridor bounded vertically
        ((Direction.UP, Direction.DOWN), (Direction.LEFT, Direction.RIGHT)),  # second orientation: vertical corridor bounded horizontally
    )

    for axis_dirs, side_dirs in orientations:  # iterate over each corridor orientation
        if any(crate + side.to_vector() in walkable for side in side_dirs):  # skip when corridor opens sideways at crate
            continue  # move to next orientation since this one is not a tunnel

        cells: Optional[Set[Point]] = {crate}  # begin collecting corridor cells starting with crate position
        for axis in axis_dirs:  # walk along both forward and backward directions
            current = crate  # reset traversal origin for this axis direction
            step = axis.to_vector()  # convert axis direction to vector offset
            while True:  # advance along corridor until blocked
                next_pos = current + step  # compute the next position along the corridor
                if next_pos not in walkable:  # stop if corridor hits a wall
                    break  # exit the loop for this axis direction
                if any(next_pos + side.to_vector() in walkable for side in side_dirs):  # detect if corridor opens sideways
                    cells = None  # mark corridor as invalid when an opening appears
                    break  # exit axis traversal because tunnel no longer closed
                cells.add(next_pos)  # record corridor square as part of dead-end path
                current = next_pos  # advance to next position for continued scanning
            if cells is None:  # after traversal, abort orientation if corridor opened
                break  # exit outer loop for this orientation

        if cells and not any(cell in layout.goals for cell in cells):  # validate corridor has no goals and remained closed
            return True  # identify crate as stuck in dead-end corridor

    return False  # after checking all orientations, no tunnel deadlock found


def _forms_2x2_deadlock(crate: Point, crates: FrozenSet[Point], layout: SokobanLayout) -> bool:  # detect 2x2 frozen squares
    if crate in layout.goals:  # ignore crate if already on goal
        return False  # no deadlock for crate placed on target

    for dx in (0, -1):  # iterate horizontal offsets to cover both 2x2 orientations
        for dy in (0, -1):  # iterate vertical offsets to cover both 2x2 orientations
            cells = [  # construct the four cells forming the considered 2x2 block
                crate + Point(dx, dy),  # top-left corner candidate
                crate + Point(dx + 1, dy),  # top-right corner candidate
                crate + Point(dx, dy + 1),  # bottom-left corner candidate
                crate + Point(dx + 1, dy + 1),  # bottom-right corner candidate
            ]
            if any(cell in layout.goals for cell in cells):  # allow configuration if block overlaps a goal
                continue  # skip to next block orientation when a goal exists inside
            if all(cell in crates or cell not in layout.walkable for cell in cells):  # verify block is fully filled or blocked
                return True  # declare frozen deadlock when 2x2 block is sealed off goals

    return False  # no 2x2 frozen pattern detected around this crate


def _hungarian_min_cost(cost_matrix: list[list[int]]) -> int:  # compute minimum cost perfect matching via Hungarian algorithm
    size = len(cost_matrix)  # determine number of crates (rows)
    if size == 0:  # handle empty matrix case
        return 0  # zero cost when no assignments needed

    dimension = len(cost_matrix[0])  # determine number of goals (columns)
    u = [0] * (size + 1)  # initialise potentials for rows
    v = [0] * (dimension + 1)  # initialise potentials for columns
    p = [0] * (dimension + 1)  # track current matching
    way = [0] * (dimension + 1)  # track augmenting path information

    for row in range(1, size + 1):  # iterate through each row to build matching
        p[0] = row  # start augmenting path with current row
        j0 = 0  # begin from dummy column
        minv = [float('inf')] * (dimension + 1)  # track minimal reduced costs
        used = [False] * (dimension + 1)  # mark visited columns
        while True:  # iterate until augmenting path closes
            used[j0] = True  # mark current column as used
            i0 = p[j0]  # get matched row for current column
            delta = float('inf')  # track best slack
            j1 = 0  # track column with minimal slack
            for col in range(1, dimension + 1):  # evaluate all columns
                if used[col]:  # skip already used columns
                    continue
                cur = cost_matrix[i0 - 1][col - 1] - u[i0] - v[col]  # compute reduced cost for edge
                if cur < minv[col]:  # update slack if smaller cost found
                    minv[col] = cur  # record new minimal reduced cost
                    way[col] = j0  # remember predecessor column along path
                if minv[col] < delta:  # keep best slack among columns
                    delta = minv[col]  # update minimal slack value
                    j1 = col  # remember column with minimal slack
            for col in range(dimension + 1):  # adjust potentials based on slack
                if used[col]:  # update for used columns
                    u[p[col]] += delta  # modify row potential per Hungarian update rule
                    v[col] -= delta  # modify column potential per Hungarian update rule
                else:  # update slack for unused columns
                    minv[col] -= delta  # reduce slack by delta for future iterations
            j0 = j1  # move to column with minimal slack
            if p[j0] == 0:  # stop when augmenting path reaches unmatched column
                break  # exit augmentation loop
        while True:  # retrace path to augment matching
            j1 = way[j0]  # get predecessor column on augmenting path
            p[j0] = p[j1]  # redirect matching to new column
            j0 = j1  # continue walking backwards
            if j0 == 0:  # stop when dummy column reached
                break  # finish augmenting path reconstruction

    assignment = [0] * (size + 1)  # allocate array to read final matching
    for col in range(1, dimension + 1):  # iterate over all columns to record matches
        if p[col]:  # only consider matched columns
            assignment[p[col]] = col  # store matched column for corresponding row

    total = 0  # initialise total cost accumulator
    for row in range(1, size + 1):  # sum costs over final assignments
        total += cost_matrix[row - 1][assignment[row] - 1]  # add cost value for matched pair
    return total  # return minimal assignment cost


def _minimum_matching_distance(crates: FrozenSet[Point], goals: FrozenSet[Point],  # compute minimum total push distance using Hungarian algorithm
                               goal_distances: Dict[Point, Dict[Point, int]]) -> float:
    if not crates:  # handle empty crate set
        return 0.0  # no pushes needed when nothing to move

    crate_list = list(crates)  # convert crates to ordered list for matrix indexing
    goal_list = list(goals)  # convert goals to ordered list for matrix indexing
    cost_matrix: list[list[int]] = []  # prepare cost matrix container
    inf_cost = 10 ** 6  # pick large sentinel cost for unreachable assignments

    for crate in crate_list:  # build one cost row per crate
        row: list[int] = []  # prepare new row for current crate
        reachable_goal = False  # track whether crate can reach any goal
        for goal in goal_list:  # iterate through each goal
            distance_map = goal_distances.get(goal, {})  # fetch pull-distance map for goal
            distance = distance_map.get(crate, inf_cost)  # obtain distance or sentinel if unreachable
            if distance < inf_cost:  # flag when crate can reach this goal
                reachable_goal = True  # indicate at least one feasible assignment exists
            row.append(distance)  # append distance to cost row
        if not reachable_goal:  # detect unsolvable crate with no reachable goals
            return float('inf')  # report deadlock via infinite heuristic
        cost_matrix.append(row)  # push constructed row into cost matrix

    return float(_hungarian_min_cost(cost_matrix))  # solve assignment and return total cost as float

def strong_heuristic(problem: SokobanProblem, state: SokobanState) -> float:  # compute advanced Sokoban heuristic with pruning
    deadlock_positions, goal_distances = _layout_info(problem)  # fetch cached deadlock squares and pull distances

    for crate in state.crates:  # iterate through every crate to detect immediate deadlocks
        if crate in deadlock_positions:  # check if crate sits on static dead square
            return float('inf')  # prune state by returning infinity
        if _in_dead_end_corridor(crate, problem.layout):  # evaluate tunnel dead-end condition
            return float('inf')  # prune tunnel deadlock states immediately
        if _forms_2x2_deadlock(crate, state.crates, problem.layout):  # test for 2x2 frozen boxes
            return float('inf')  # prune frozen block configurations

    matching_distance = _minimum_matching_distance(state.crates, problem.layout.goals, goal_distances)  # compute minimum push matching cost
    if matching_distance == float('inf'):  # exit if matching indicates unsolvable crate placement
        return float('inf')  # maintain pruning semantics for unreachable crates
    if matching_distance == 0.0:  # shortcut when all crates already placed correctly
        return 0.0  # return zero heuristic at solved states

    player_to_crate = min(manhattan_distance(state.player, crate) for crate in state.crates)  # measure player distance to closest crate
    heuristic_value = max(matching_distance, player_to_crate)  # combine crate and player bounds conservatively using max
    return float(heuristic_value)  # return final heuristic as floating-point value
