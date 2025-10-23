from problem import HeuristicFunction, Problem, S, A, Solution
from collections import deque
from helpers.utils import NotImplemented

#TODO: Import any modules you want to use
from heapq import heappush, heappop
import itertools
# import heapq

# All search functions take a problem and a state
# If it is an informed search function, it will also receive a heuristic function
# S and A are used for generic typing where S represents the state type and A represents the action type

# All the search functions should return one of two possible type:
# 1. A list of actions which represent the path from the initial state to the final state
# 2. None if there is no solution

def get_solution(parent_map,state):
    solution = [state]
    while(parent_map[state]):
        state = parent_map[state]
        solution.append(state)
    solution.pop()
    solution.reverse()
    return solution

def BreadthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    #TODO: ADD YOUR CODE HERE
    frontier = deque()
    explored = set()
    parent_map = {initial_state:None}
    frontier.append(initial_state)
    while frontier:
        state = frontier.popleft()
        explored.add(state)
        
        for action in problem.get_actions(state):
            next_state = problem.get_successor(state,action)
            if(problem.is_goal(next_state)):
                parent_map[next_state] = state
                return get_solution(parent_map,next_state)
            if next_state not in frontier and next_state not in explored:
                parent_map[next_state] = state
                frontier.append(next_state)

    return None
    # NotImplemented()

def DepthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    #TODO: ADD YOUR CODE HERE
    stack = []
    explored = set()
    parent_map = {initial_state:None}
    stack.append(initial_state)
    while stack:
        state = stack.pop()
        explored.add(state)
        
        for action in problem.get_actions(state):
            next_state = problem.get_successor(state,action)
            if(problem.is_goal(next_state)):
                parent_map[next_state] = state
                return get_solution(parent_map,next_state)
            if next_state not in stack and next_state not in explored:
                parent_map[next_state] = state
                stack.append(next_state)

    return None
    # NotImplemented()
    

def UniformCostSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    frontier = []
    g_costs = {initial_state: 0}
    counter = itertools.count()
    heappush(frontier, (0, next(counter), initial_state))  # (cost, state)
    explored = set()
    parent_map = {initial_state: None}

    while frontier:
        cost, _, state = heappop(frontier)   # lowest-cost node
        if state in explored:
            continue
        explored.add(state)

        if problem.is_goal(state):
            return get_solution(parent_map, state)

        for action in problem.get_actions(state):
            next_state = problem.get_successor(state, action)
            new_cost = cost + problem.get_cost(state, action)
            if next_state not in g_costs or new_cost < g_costs[next_state]:
                g_costs[next_state] = new_cost
                heappush(frontier, (new_cost, next(counter), next_state))
                parent_map[next_state] = state
    return None

    # NotImplemented()

def AStarSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    #TODO: ADD YOUR CODE HERE
    frontier = []
    hn = heuristic(problem,initial_state)
    fns = {initial_state:hn}
    g_costs = {initial_state: 0}
    counter = itertools.count()
    heappush(frontier, (hn, next(counter), initial_state))  # (cost, state)
    explored = set()
    parent_map = {initial_state: None}

    while frontier:
        cost, _, state = heappop(frontier)   # lowest-cost node
        if state in explored:
            continue
        explored.add(state)

        if problem.is_goal(state):
            return get_solution(parent_map, state)

        for action in problem.get_actions(state):
            next_state = problem.get_successor(state, action)
            new_cost = g_costs[state] + problem.get_cost(state, action)
            hn = heuristic(problem,next_state)
            new_fn = new_cost + hn
            if next_state not in fns or new_fn < fns[next_state]:
                fns[next_state] = new_fn
                g_costs[next_state] = new_cost
                heappush(frontier, (new_fn, next(counter), next_state))
                parent_map[next_state] = state
    return None

    # NotImplemented()

def BestFirstSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    frontier = []
    h0 = heuristic(problem,initial_state)
    heuristics = {initial_state:h0}
    counter = itertools.count()
    heappush(frontier, (h0, next(counter), initial_state))  # (cost, state)
    explored = set()
    parent_map = {initial_state: None}

    while frontier:
        h_val, _, state = heappop(frontier)   # lowest-cost node
        if state in explored:
            continue
        explored.add(state)

        if problem.is_goal(state):
            return get_solution(parent_map, state)

        for action in problem.get_actions(state):
            next_state = problem.get_successor(state, action)
            if next_state in explored:
                continue
            if next_state in heuristics:
                new_heuristic = heuristics[next_state]
            else:
                new_heuristic = heuristic(problem,next_state)
                heuristics[next_state] = new_heuristic
            heappush(frontier, (new_heuristic, next(counter), next_state))
            parent_map[next_state] = state
    return None