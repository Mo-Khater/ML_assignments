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

def get_solution(parent_map, action_map, state):
    # This little helper lets me walk backwards from the goal while grumbling softley about pointers.
    actions = []  # the solution is a list of states including all states execpt the initial one 
    while(parent_map[state]): # while the state has a parent
        actions.append(action_map[state])
        state = parent_map[state] 

    actions.reverse() # reverse the solution to be from initial to goal
    return actions

def BreadthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    #TODO: ADD YOUR CODE HERE
    frontier = deque() # use queue for BFS
    explored = set() # use explore to use graph search
    parent_map = {initial_state:None} # used to store the parents
    action_map = {initial_state: None}  # used to store actions
    frontier.append(initial_state)
    while frontier: # as the frontier isn't empty
        state = frontier.popleft() # get the first pushed state
        explored.add(state) # add it to explored
        
        for action in problem.get_actions(state): # for each action applicable to the state
            next_state = problem.get_successor(state,action) # get the next state
            if(problem.is_goal(next_state)): # if it is the goal return the solution
                parent_map[next_state] = state 
                action_map[next_state] = action
                return get_solution(parent_map, action_map, next_state) 
            if next_state not in frontier and next_state not in explored: # if it isn't in frontier or explored add it to frontier
                parent_map[next_state] = state 
                action_map[next_state] = action
                frontier.append(next_state) 

    return None

def DepthFirstSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    #TODO: ADD YOUR CODE HERE
    stack = [] # use stack for DFS
    explored = set() # use explore to use graph search
    parent_map = {initial_state:None} # used to store the parents
    action_map = {initial_state: None}  # used to store actions
    stack.append(initial_state) 
    while stack: # as the stack isn't empty
        state = stack.pop() # get the last pushed state
        explored.add(state) # add it to explored

        if(problem.is_goal(state)):    # if it is the goal return the solution
            return get_solution(parent_map, action_map, state)

        for action in problem.get_actions(state): # for each action applicable to the state
            next_state = problem.get_successor(state,action) # get the next state
            if next_state not in stack and next_state not in explored: # if it isn't in stack or explored add it to stack
                parent_map[next_state] = state 
                action_map[next_state] = action
                stack.append(next_state) 

    return None
    # NotImplemented()
    

def UniformCostSearch(problem: Problem[S, A], initial_state: S) -> Solution:
    frontier = [] # use priority queue for UCS
    current_costs = {initial_state: 0} # store the g costs for each state
    counter = itertools.count() # if tie happen
    heappush(frontier, (0, next(counter), initial_state))  # push the initial state with cost 0
    explored = set() # use explore to use graph search
    parent_map = {initial_state: None} # used to store the parents
    action_map = {initial_state: None}  # used to store actions

    while frontier: 
        cost, _, state = heappop(frontier)   # lowest-cost node
        if state in explored: # if already explored
            continue
        explored.add(state) # add to explored

        if problem.is_goal(state): # if it is the goal
            return get_solution(parent_map, action_map, state)

        for action in problem.get_actions(state): # for each action applicable to the state
            next_state = problem.get_successor(state, action) # get the next state
            new_cost = cost + problem.get_cost(state, action) # calculate the new cost
            if next_state not in current_costs or new_cost < current_costs[next_state]: # if it isn't in costs or the new cost is less
                current_costs[next_state] = new_cost # update the cost
                heappush(frontier, (new_cost, next(counter), next_state)) # push to frontier
                parent_map[next_state] = state # set the parent
                action_map[next_state] = action
    return None

    # NotImplemented()

def AStarSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    #TODO: ADD YOUR CODE HERE
    frontier = [] # use priority queue for A*
    hn = heuristic(problem,initial_state) # calculate h for the initial state
    fn0 = hn  # f(n) = g(n) + h(n) for initial state
    fns = {initial_state: fn0} # store the f(n) costs for each state
    current_costs = {initial_state: 0} # store the g(n) costs for each state
    counter = itertools.count() # if tie happen 
    heappush(frontier, (fn0, next(counter), initial_state)) 
    explored = set() # use explore to use graph search
    parent_map = {initial_state: None}
    action_map = {initial_state: None}  # used to store actions

    while frontier:
        cost, _, state = heappop(frontier)   # lowest f(n) node
        if state in explored:
            continue
        explored.add(state)

        if problem.is_goal(state):
            return get_solution(parent_map, action_map, state)

        for action in problem.get_actions(state): # for each action applicable to the state
            next_state = problem.get_successor(state, action) # get the next state
            if next_state in explored:
                continue
            new_cost = current_costs[state] + problem.get_cost(state, action) # calculate g(n)
            hn = heuristic(problem,next_state) # calculate h(n)
            new_fn = new_cost + hn # calculate f(n)
            if next_state not in fns or new_fn < fns[next_state]: # if it isn't in fns or the new f(n) is less
                fns[next_state] = new_fn # update f(n)
                current_costs[next_state] = new_cost # update g(n)
                heappush(frontier, (new_fn, next(counter), next_state)) # push to frontier
                parent_map[next_state] = state # set the parent
                action_map[next_state] = action
    return None

    # NotImplemented()

def BestFirstSearch(problem: Problem[S, A], initial_state: S, heuristic: HeuristicFunction) -> Solution:
    frontier = [] # use priority queue for Best-First Search
    h0 = heuristic(problem,initial_state) # calculate h for the initial state
    heuristics = {initial_state:h0} # store the h0 for the inital state
    frontier_states = {initial_state} # track states currently in the frontier
    counter = itertools.count()  # if tie happen
    heappush(frontier, (h0, next(counter), initial_state)) 
    explored = set() # use explore to use graph search
    parent_map = {initial_state: None}
    action_map = {initial_state: None}  # used to store actions

    while frontier:
        h_val, _, state = heappop(frontier)   # lowest-cost node
        if state in explored:
            continue
        explored.add(state)
        frontier_states.discard(state)

        if problem.is_goal(state):
            return get_solution(parent_map, action_map, state)

        for action in problem.get_actions(state): # for each action applicable to the state
            next_state = problem.get_successor(state, action) # get the next state
            if next_state in explored or next_state in frontier_states: 
                continue
            if next_state in heuristics: # if already calculated
                new_heuristic = heuristics[next_state] 
            else:
                new_heuristic = heuristic(problem,next_state) # calculate h(n)
                heuristics[next_state] = new_heuristic # store h(n)
            heappush(frontier, (new_heuristic, next(counter), next_state)) # push to frontier
            frontier_states.add(next_state)
            parent_map[next_state] = state # set the parent
            action_map[next_state] = action
    return None