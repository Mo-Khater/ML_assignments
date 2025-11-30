from typing import Tuple
from game import HeuristicFunction, Game, S, A
from helpers.utils import NotImplemented

#TODO: Import any modules you want to use

# All search functions take a problem, a state, a heuristic function and the maximum search depth.
# If the maximum search depth is -1, then there should be no depth cutoff (The expansion should not stop before reaching a terminal state) 

# All the search functions should return the expected tree value and the best action to take based on the search results

# This is a simple search function that looks 1-step ahead and returns the action that lead to highest heuristic value.
# This algorithm is bad if the heuristic function is weak. That is why we use minimax search to look ahead for many steps.
def greedy(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    agent = game.get_turn(state)
    
    terminal, values = game.is_terminal(state)
    if terminal: return values[agent], None

    actions_states = [(action, game.get_successor(state, action)) for action in game.get_actions(state)]
    value, _, action = max((heuristic(game, state, agent), -index, action) for index, (action , state) in enumerate(actions_states))
    return value, action

# Apply Minimax search and return the game tree value and the best action
# Hint: There may be more than one player, and in all the testcases, it is guaranteed that 
# game.get_turn(state) will return 0 (which means it is the turn of the player). All the other players
# (turn > 0) will be enemies. So for any state "s", if the game.get_turn(s) == 0, it should a max node,
# and if it is > 0, it should be a min node. Also remember that game.is_terminal(s), returns the values
# for all the agents. So to get the value for the player (which acts at the max nodes), you need to
# get values[0].
def minimax(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    agent = game.get_turn(state) # Get current agent's turn
    
    # Check terminal condition
    terminal, values = game.is_terminal(state) 
    if terminal: # If terminal state,
        return values[0], None # return value for player (agent 0)
    
    # Check depth limit
    if max_depth == 0: 
        return heuristic(game, state, 0), None # Return heuristic value for player (agent 0)
    
    # Maximizing player (agent 0)
    if agent == 0:
        max_value = float('-inf')
        best_action = None
        for action in game.get_actions(state): # Iterate through possible actions
            successor = game.get_successor(state, action)  # Get next state
            value, _ = minimax(game, successor, heuristic, max_depth - 1) 
            if value > max_value: # take the max for max node
                max_value = value 
                best_action = action
        return max_value, best_action
    
    # Minimizing player
    else:
        min_value = float('inf')
        best_action = None
        for action in game.get_actions(state): # Iterate through possible actions
            successor = game.get_successor(state, action)  # Get next state
            value, _ = minimax(game, successor, heuristic, max_depth - 1)
            if value < min_value:  # take the min for min node
                min_value = value  
                best_action = action
        return min_value, best_action  


# Apply Alpha Beta pruning and return the tree value and the best action
# Hint: Read the hint for minimax.
def alphabeta(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    #TODO: Complete this function
    agent = game.get_turn(state) # Get current agent's turn
    alpha = float('-inf')
    beta = float('inf')

    def ab_helper(state: S, depth: int, alpha: float, beta: float) -> Tuple[float, A]:
        agent = game.get_turn(state)
        
        # Check terminal condition
        terminal, values = game.is_terminal(state) 
        if terminal: # If terminal state,
            return values[0], None # return value for player (agent 0)
        
        # Check depth limit
        if depth == 0: 
            return heuristic(game, state, 0), None # Return heuristic value for player (agent 0)
        
        # Maximizing player (agent 0)
        if agent == 0:
            max_value = float('-inf')
            best_action = None
            for action in game.get_actions(state): # Iterate through possible actions
                successor = game.get_successor(state, action)  # Get next state
                value, _ = ab_helper(successor, depth - 1, alpha, beta) 
                if value > max_value: # take the max for max node
                    max_value = value 
                    best_action = action
                if beta <= max_value:
                    return max_value,None  # Beta cut-off
                alpha = max(alpha, max_value)
            return max_value, best_action
        
        # Minimizing player
        else:
            min_value = float('inf')
            best_action = None
            for action in game.get_actions(state): # Iterate through possible actions
                successor = game.get_successor(state, action)  # Get next state
                value, _ = ab_helper(successor, depth - 1, alpha, beta)
                if value < min_value:  # take the min for min node
                    min_value = value  
                    best_action = action
                if min_value <= alpha:
                    return min_value,None  # Alpha cut-off
                beta = min(beta, min_value)
            return min_value, best_action
    
    return ab_helper(state, max_depth, alpha, beta)

# Apply Alpha Beta pruning with move ordering and return the tree value and the best action
# Hint: Read the hint for minimax.
def alphabeta_with_move_ordering(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    #TODO: Complete this function
    agent = game.get_turn(state) # Get current agent's turn
    alpha = float('-inf')
    beta = float('inf')
    def abmo_helper(state: S, depth: int, alpha: float, beta: float) -> Tuple[float, A]:
        agent = game.get_turn(state)
        
        # Check terminal condition
        terminal, values = game.is_terminal(state) 
        if terminal: # If terminal state,
            return values[0], None # return value for player (agent 0)
        
        # Check depth limit
        if depth == 0: 
            return heuristic(game, state, 0), None # Return heuristic value for player (agent 0)
        
        actions = game.get_actions(state)
        
        # Order actions based on heuristic values
        action_values = []
        for i,action in enumerate(actions):
            successor = game.get_successor(state, action)
            h_value = heuristic(game, successor, 0)
            action_values.append((h_value, action,i))
        action_values.sort(key=lambda x:(-x[0] if agent == 0 else x[0],x[2]))  # Sort descending for max node, ascending for min node based on heuristic value and original index
        ordered_actions = [action for _, action,_ in action_values]
        
        # Maximizing player (agent 0)
        if agent == 0:
            max_value = float('-inf')
            best_action = None
            for action in ordered_actions: # Iterate through ordered actions
                successor = game.get_successor(state, action)  # Get next state
                value, _ = abmo_helper(successor, depth - 1, alpha, beta) 
                if value > max_value: # take the max for max node
                    max_value = value 
                    best_action = action
                if beta <= max_value:
                    return max_value,None  # Beta cut-off
                alpha = max(alpha, max_value)
            return max_value, best_action
        
        # Minimizing player
        else:
            min_value = float('inf')
            best_action = None
            for action in ordered_actions: # Iterate through ordered actions
                successor = game.get_successor(state, action)  # Get next state
                value, _ = abmo_helper(successor, depth - 1, alpha, beta)
                if value < min_value:  # take the min for min node
                    min_value = value  
                    best_action = action
                if min_value <= alpha:
                    return min_value,None  # Alpha cut-off
                beta = min(beta, min_value)
            return min_value, best_action
        
    return abmo_helper(state, max_depth, alpha, beta)

# Apply Expectimax search and return the tree value and the best action
# Hint: Read the hint for minimax, but note that the monsters (turn > 0) do not act as min nodes anymore,
# they now act as chance nodes (they act randomly).
def expectimax(game: Game[S, A], state: S, heuristic: HeuristicFunction, max_depth: int = -1) -> Tuple[float, A]:
    #TODO: Complete this function
    agent = game.get_turn(state) # Get current agent's turn
    # Check terminal condition
    terminal, values = game.is_terminal(state)
    if terminal: # If terminal state,
        return values[0], None # return value for player (agent 0)
    # Check depth limit
    if max_depth == 0:
        return heuristic(game, state, 0), None # Return heuristic value for player (agent 0)
    # Maximizing player (agent 0)
    if agent == 0:
        max_value = float('-inf')
        best_action = None
        for action in game.get_actions(state): # Iterate through possible actions
            successor = game.get_successor(state, action)  # Get next state
            value, _ = expectimax(game, successor, heuristic, max_depth - 1)
            if value > max_value: # take the max for max node
                max_value = value
                best_action = action
        return max_value, best_action
    
    # Chance node (monsters)
    else:
        total_value = 0.0
        actions = game.get_actions(state)
        for action in actions:  # Iterate through possible actions
            successor = game.get_successor(state, action)  # Get next state
            value, _ = expectimax(game, successor, heuristic, max_depth - 1)
            total_value += value / len(actions)  # Expected value calculation
        return total_value, None
    
    