from sokoban import SokobanProblem, SokobanState
from mathutils import Direction, Point, manhattan_distance
from helpers.utils import NotImplemented
from collections import deque

# This heuristic returns the distance between the player and the nearest crate as an estimate for the path cost
# While it is consistent, it does a bad job at estimating the actual cost thus the search will explore a lot of nodes before finding a goal
def weak_heuristic(problem: SokobanProblem, state: SokobanState):
    return min(manhattan_distance(state.player, crate) for crate in state.crates) - 1

#TODO: Import any modules and write any functions you want to use


def build_goal_distance_database(problem: SokobanProblem):
    """Pre-compute distances from every position to each goal separately"""
    cache = problem.cache()
    
    if 'goal_distances' in cache:
        return cache['goal_distances']
    
    goal_distances = {}
    
    # For each goal, compute distance to all reachable positions
    for goal in problem.layout.goals:
        distances = {goal: 0}
        queue = deque([goal])
        
        while queue:
            pos = queue.popleft()
            current_dist = distances[pos]
            
            # Check all 4 directions
            for direction in [Direction.UP, Direction.DOWN, Direction.LEFT, Direction.RIGHT]:
                next_pos = pos + direction.to_vector()
                
                if next_pos in problem.layout.walkable and next_pos not in distances:
                    distances[next_pos] = current_dist + 1
                    queue.append(next_pos)
        
        goal_distances[goal] = distances
    
    cache['goal_distances'] = goal_distances
    return goal_distances

def detect_corner_deadlock(problem: SokobanProblem, crate: Point) -> bool:
    """Check if crate is stuck in a corner (simple deadlock detection)"""
    if crate in problem.layout.goals:
        return False
    
    layout = problem.layout
    up = crate + Direction.UP.to_vector()
    down = crate + Direction.DOWN.to_vector()
    left = crate + Direction.LEFT.to_vector()
    right = crate + Direction.RIGHT.to_vector()
    
    # Check if crate is blocked vertically and horizontally
    vertical_blocked = (up not in layout.walkable or down not in layout.walkable)
    horizontal_blocked = (left not in layout.walkable or right not in layout.walkable)
    
    return vertical_blocked and horizontal_blocked

def detect_wall_deadlock(problem: SokobanProblem, crate: Point) -> bool:
    """Check if crate is stuck along a wall with no goals - simplified fast version"""
    if crate in problem.layout.goals:
        return False
    
    layout = problem.layout
    goals_set = problem.layout.goals
    
    # Simple check: if surrounded by walls with no adjacent goal
    up = crate + Direction.UP.to_vector()
    down = crate + Direction.DOWN.to_vector()
    left = crate + Direction.LEFT.to_vector()
    right = crate + Direction.RIGHT.to_vector()
    
    adjacent_goals = sum(1 for pos in [up, down, left, right] if pos in goals_set)
    
    # If no adjacent goals and we're boxed in, it's a deadlock
    if adjacent_goals == 0:
        walls = sum(1 for pos in [up, down, left, right] if pos not in layout.walkable)
        # If 2+ walls and only 2 or fewer walkable spaces, likely deadlock
        if walls >= 2:
            return True
    
    return False

def detect_line_deadlock(problem: SokobanProblem, crate: Point) -> bool:
    """Check if crate is stuck in a line with no goals nearby"""
    if crate in problem.layout.goals:
        return False
    
    layout = problem.layout
    goals_set = problem.layout.goals
    
    # Check if stuck horizontally (wall on both left and right, no goal)
    left = crate + Direction.LEFT.to_vector()
    right = crate + Direction.RIGHT.to_vector()
    
    if left not in layout.walkable and right not in layout.walkable:
        # Stuck horizontally, check if any goal nearby
        up = crate + Direction.UP.to_vector()
        down = crate + Direction.DOWN.to_vector()
        if up not in goals_set and down not in goals_set:
            return True
    
    # Check if stuck vertically (wall on both up and down, no goal)
    up = crate + Direction.UP.to_vector()
    down = crate + Direction.DOWN.to_vector()
    
    if up not in layout.walkable and down not in layout.walkable:
        # Stuck vertically, check if any goal nearby
        left = crate + Direction.LEFT.to_vector()
        right = crate + Direction.RIGHT.to_vector()
        if left not in goals_set and right not in goals_set:
            return True
    
    return False

def detect_frozen_deadlock(problem: SokobanProblem, state: SokobanState, crate: Point) -> bool:
    """Check if crate can't be pushed to any goal (surrounded by other crates or walls)"""
    if crate in problem.layout.goals:
        return False
    
    layout = problem.layout
    
    # Check all 4 directions - if surrounded by walls or other crates, it's frozen
    directions = [Direction.UP, Direction.DOWN, Direction.LEFT, Direction.RIGHT]
    blocked = 0
    
    for direction in directions:
        next_pos = crate + direction.to_vector()
        if next_pos not in layout.walkable or next_pos in state.crates:
            blocked += 1
    
    # If 3 or more directions blocked, crate is frozen
    return blocked >= 3

def strong_heuristic(problem: SokobanProblem, state: SokobanState) -> float:
    if not state.crates:
        return 0
    
    cache = problem.cache()
    
    # Cache goals set, list, database, deadlock positions, and state heuristic results
    if 'goals_set' not in cache:
        cache['goals_set'] = problem.layout.goals
        cache['goals_list'] = list(problem.layout.goals)
        cache['goal_distances'] = build_goal_distance_database(problem)
        cache['position_deadlock'] = {}  # Cache deadlock status for each position
        cache['state_heuristics'] = {}
    
    goals_set = cache['goals_set']
    goals_list = cache['goals_list']
    goal_distances = cache['goal_distances']
    position_deadlock = cache['position_deadlock']
    state_heuristics = cache['state_heuristics']
    
    # Quick check: if all crates are on goals
    if state.crates.issubset(goals_set):
        return 0
    
    # Use state as key for caching (frozenset is hashable)
    state_key = state.crates
    
    # Check if we've already calculated heuristic for this state
    if state_key in state_heuristics:
        return state_heuristics[state_key]
    
    # Sum of minimum distances for unplaced crates
    total = 0
    for crate in state.crates:
        if crate in goals_set:
            continue
        
        # Check cached deadlock status for this position
        if crate not in position_deadlock:
            # Calculate and cache deadlock status
            if (detect_corner_deadlock(problem, crate) or
                detect_wall_deadlock(problem, crate) or
                detect_line_deadlock(problem, crate)):
                position_deadlock[crate] = True
            else:
                position_deadlock[crate] = False
        
        # Check static deadlock (doesn't depend on state)
        if position_deadlock[crate]:
            state_heuristics[state_key] = float('inf')
            return float('inf')
        
        # Check frozen deadlock (depends on current state)
        if detect_frozen_deadlock(problem, state, crate):
            state_heuristics[state_key] = float('inf')
            return float('inf')
        
        # Lookup pre-computed distance
        dist = goal_distances.get(crate)
        if dist is not None:
            total += dist
        else:
            # Fallback to Manhattan
            total += min(manhattan_distance(crate, goal) for goal in goals_list)
    
    # Cache the result
    state_heuristics[state_key] = total
    return total