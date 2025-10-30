from sokoban import SokobanProblem, SokobanState, SokobanLayout
from mathutils import Direction, Point, manhattan_distance
from helpers.utils import NotImplemented
from collections import deque

# This heuristic returns the distance between the player and the nearest crate as an estimate for the path cost
# While it is consistent, it does a bad job at estimating the actual cost thus the search will explore a lot of nodes before finding a goal
def weak_heuristic(problem: SokobanProblem, state: SokobanState):
    return min(manhattan_distance(state.player, crate) for crate in state.crates) - 1

#TODO: Import any modules and write any functions you want to use

_deadlock_cache = {}

def get_deadlock_positions(layout: SokobanLayout) -> frozenset[Point]:
    """
    Identifies and caches deadlock positions for a given Sokoban layout.
    A deadlock position is a non-goal square from which a crate cannot be moved to any goal.
    This implementation detects simple "corner" and "edge" deadlocks.
    """
    if layout in _deadlock_cache:
        return _deadlock_cache[layout]

    deadlocks = set()
    for y in range(layout.height):
        for x in range(layout.width):
            p = Point(x, y)
            if p in layout.goals or p not in layout.walkable:
                continue

            # Check for corner deadlocks
            is_deadlock = False
            # Up-Left
            if (p + Direction.UP.to_vector() not in layout.walkable and
                p + Direction.LEFT.to_vector() not in layout.walkable):
                is_deadlock = True
            # Up-Right
            if (p + Direction.UP.to_vector() not in layout.walkable and
                p + Direction.RIGHT.to_vector() not in layout.walkable):
                is_deadlock = True
            # Down-Left
            if (p + Direction.DOWN.to_vector() not in layout.walkable and
                p + Direction.LEFT.to_vector() not in layout.walkable):
                is_deadlock = True
            # Down-Right
            if (p + Direction.DOWN.to_vector() not in layout.walkable and
                p + Direction.RIGHT.to_vector() not in layout.walkable):
                is_deadlock = True
            
            if is_deadlock:
                deadlocks.add(p)

    # Check for edge deadlocks
    for p in list(deadlocks): # Iterate over a copy
        # If a crate is on an edge and can't slide off, it's a deadlock
        # Check horizontal edge
        if (p + Direction.UP.to_vector() not in layout.walkable or 
            p + Direction.DOWN.to_vector() not in layout.walkable):
            # on a horizontal edge, check if it can slide out
            can_slide = False
            # check left
            temp_p = p
            while temp_p in layout.walkable and (temp_p + Direction.UP.to_vector() not in layout.walkable or temp_p + Direction.DOWN.to_vector() not in layout.walkable):
                if temp_p in layout.goals:
                    can_slide = True
                    break
                temp_p += Direction.LEFT.to_vector()
            # check right
            if not can_slide:
                temp_p = p
                while temp_p in layout.walkable and (temp_p + Direction.UP.to_vector() not in layout.walkable or temp_p + Direction.DOWN.to_vector() not in layout.walkable):
                    if temp_p in layout.goals:
                        can_slide = True
                        break
                    temp_p += Direction.RIGHT.to_vector()
            if not can_slide:
                deadlocks.add(p)

        # Check vertical edge
        if (p + Direction.LEFT.to_vector() not in layout.walkable or 
            p + Direction.RIGHT.to_vector() not in layout.walkable):
            # on a vertical edge, check if it can slide out
            can_slide = False
            # check up
            temp_p = p
            while temp_p in layout.walkable and (temp_p + Direction.LEFT.to_vector() not in layout.walkable or temp_p + Direction.RIGHT.to_vector() not in layout.walkable):
                if temp_p in layout.goals:
                    can_slide = True
                    break
                temp_p += Direction.UP.to_vector()
            # check down
            if not can_slide:
                temp_p = p
                while temp_p in layout.walkable and (temp_p + Direction.LEFT.to_vector() not in layout.walkable or temp_p + Direction.RIGHT.to_vector() not in layout.walkable):
                    if temp_p in layout.goals:
                        can_slide = True
                        break
                    temp_p += Direction.DOWN.to_vector()
            if not can_slide:
                deadlocks.add(p)


    _deadlock_cache[layout] = frozenset(deadlocks)
    return _deadlock_cache[layout]


def strong_heuristic(problem: SokobanProblem, state: SokobanState) -> float:
    """
    Implements a strong heuristic for Sokoban using a combination of deadlock
    detection and the "Simple Lower Bound" method.

    1. Deadlock Pruning: It first checks if any crate is in a pre-calculated
       deadlock position. If so, it returns infinity to prune this state.

    2. Simple Lower Bound: If no deadlocks are found, it calculates the sum
       of the Manhattan distances for each crate to its nearest goal. This
       heuristic is admissible and consistent.

    Reference: http://sokobano.de/wiki/index.php?title=Solver
    """
    
    # Pruning via Deadlock Detection
    deadlock_positions = get_deadlock_positions(problem.layout)
    for crate in state.crates:
        if crate in deadlock_positions:
            return float('inf')

    # Simple Lower Bound
    total_distance = 0
    for crate in state.crates:
        min_dist_to_goal = min(manhattan_distance(crate, goal) for goal in problem.layout.goals)
        total_distance += min_dist_to_goal
    
    return float(total_distance)
  