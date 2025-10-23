from typing import Any, Dict, Set, Tuple, List
from problem import Problem
from mathutils import Direction, Point
from helpers.utils import NotImplemented

#TODO: (Optional) Instead of Any, you can define a type for the parking state
ParkingState = Any

# An action of the parking problem is a tuple containing an index 'i' and a direction 'd' where car 'i' should move in the direction 'd'.
ParkingAction = Tuple[int, Direction]

# This is the implementation of the parking problem
class ParkingProblem(Problem[ParkingState, ParkingAction]):
    passages: Set[Point]    # A set of points which indicate where a car can be (in other words, every position except walls).
    cars: Tuple[Point]      # A tuple of points where state[i] is the position of car 'i'. 
    slots: Dict[Point, int] # A dictionary which indicate the index of the parking slot (if it is 'i' then it is the lot of car 'i') for every position.
                            # if a position does not contain a parking slot, it will not be in this dictionary.
    width: int              # The width of the parking lot.
    height: int             # The height of the parking lot.

    # This function should return the initial state
    def get_initial_state(self) -> ParkingState:
        #TODO: ADD YOUR CODE HERE
        return self.cars
        # NotImplemented()
    
    # This function should return True if the given state is a goal. Otherwise, it should return False.
    def is_goal(self, state: ParkingState) -> bool:
        #TODO: ADD YOUR CODE HERE
        for car_pos, car_index in self.slots.items(): # loop over the final positions that cars need to be in.
            if(state[car_index] != car_pos): # if the final position of car number car_index isn't equal the true pos return false
                return False 
        return True 
        # NotImplemented()
    def move(self,car,index,direction,available_actions,state):
        new_pos = Point(car.x + direction.to_vector().x,car.y + direction.to_vector().y)
        if new_pos in self.passages and new_pos not in state:
            available_actions.append((index, direction))

    # This function returns a list of all the possible actions that can be applied to the given state
    def get_actions(self, state: ParkingState) -> List[ParkingAction]:
        #TODO: ADD YOUR CODE HERE
        available_actions : List[ParkingAction] = []
        for index,car_position in enumerate(state): # for each car try to move it in any four directions
            self.move(car_position,index,Direction.RIGHT,available_actions,state)
            self.move(car_position,index,Direction.LEFT,available_actions,state)
            self.move(car_position,index,Direction.UP,available_actions,state)
            self.move(car_position,index,Direction.DOWN,available_actions,state)

        return available_actions

        # NotImplemented()
    
    # This function returns a new state which is the result of applying the given action to the given state
    def get_successor(self, state: ParkingState, action: ParkingAction) -> ParkingState:

        car_index, direction = action        
        car_position = state[car_index]
        move_vector = direction.to_vector()
        new_position = Point(car_position.x + move_vector.x, car_position.y + move_vector.y)
        new_cars = list(state)
        new_cars[car_index] = new_position
        new_cars = tuple(new_cars)

        return new_cars

        # NotImplemented()
    
    # This function returns the cost of applying the given action to the given state
    def get_cost(self, state: ParkingState, action: ParkingAction) -> float:
        #TODO: ADD YOUR CODE HERE
        return 26 - action[0]
    
     # Read a parking problem from text containing a grid of tiles
    @staticmethod
    def from_text(text: str) -> 'ParkingProblem':
        passages =  set()
        cars, slots = {}, {}
        lines = [line for line in (line.strip() for line in text.splitlines()) if line]
        width, height = max(len(line) for line in lines), len(lines)
        for y, line in enumerate(lines):
            for x, char in enumerate(line):
                if char != "#":
                    passages.add(Point(x, y))
                    if char == '.':
                        pass
                    elif char in "ABCDEFGHIJ":
                        cars[ord(char) - ord('A')] = Point(x, y)
                    elif char in "0123456789":
                        slots[int(char)] = Point(x, y)
        problem = ParkingProblem()
        problem.passages = passages
        problem.cars = tuple(cars[i] for i in range(len(cars)))
        problem.slots = {position:index for index, position in slots.items()}
        problem.width = width
        problem.height = height
        return problem

    # Read a parking problem from file containing a grid of tiles
    @staticmethod
    def from_file(path: str) -> 'ParkingProblem':
        with open(path, 'r') as f:
            return ParkingProblem.from_text(f.read())
    
