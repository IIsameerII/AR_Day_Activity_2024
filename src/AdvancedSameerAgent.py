from src.DriveInterface import DriveInterface
from src.DriveState import DriveState
from src.Constants import DriveMove, SensorData
from collections import deque

class AdvancedSameerAgent(DriveInterface):

    def __init__(self, game_id: int, is_advanced_mode: bool):
        self.game_id = game_id
        self.path = []
        self.path_move_index = 0
        self.need_to_find_target_pod = is_advanced_mode
        self.is_carrying_pod = False  # Track if the player is carrying the pod

    def get_next_move(self, sensor_data: dict) -> DriveMove:
        # Main function called by game orchestrator
        if len(self.path) == 0:
            if self.need_to_find_target_pod:
                if not self.is_carrying_pod:
                    # Phase 1: Move to target pod
                    self.bfs_solve_path_to_target_pod(sensor_data)
                else:
                    # Phase 3: Move to closest goal with pod
                    self.bfs_solve_path_to_closest_goal(sensor_data)
            else:
                # Standard mode: move directly to the closest goal
                self.bfs_solve_path_to_closest_goal(sensor_data)

        next_move, next_state = self.get_move_for_next_state_in_path()

        if self.will_next_state_collide(next_state, sensor_data):
            self.path_move_index -= 1
            return DriveMove.NONE
        elif not self.is_carrying_pod and self.is_at_target_pod(sensor_data):
            # Phase 2: Lift pod when reaching the target pod location
            self.is_carrying_pod = True
            self.path = []  # Reset path to calculate new path to goal
            return DriveMove.LIFT_POD
        elif self.is_carrying_pod and self.is_at_goal(sensor_data):
            # Phase 4: Drop pod when reaching the goal location
            self.is_carrying_pod = False
            self.path = []  # Reset path after dropping pod
            return DriveMove.DROP_POD
        else:
            return next_move

    def will_next_state_collide(self, state: DriveState, sensor_data: dict) -> bool:
        # Check for collision with any field boundaries or other drive locations
        if [state.x, state.y] in sensor_data[SensorData.FIELD_BOUNDARIES]:
            return True
        if [state.x, state.y] in sensor_data[SensorData.DRIVE_LOCATIONS]:
            return True
        return False

    def get_move_for_next_state_in_path(self) -> DriveMove:
        # Find the move which will get the player to the next state in self.path
        current_state = self.path[self.path_move_index]
        self.path_move_index += 1
        next_state = self.path[self.path_move_index]

        for move in DriveMove:
            if current_state.get_next_state_from_move(move) == next_state.to_tuple():
                return move, next_state

        print('WARN: Next move could not be found')
        return DriveMove.NONE, next_state

    def bfs_solve_path_to_target_pod(self, sensor_data: dict):
        # BFS to find the shortest path to the target pod
        start_state = sensor_data[SensorData.PLAYER_LOCATION]
        target_pod_location = sensor_data[SensorData.TARGET_POD_LOCATION]

        self.path = self.bfs_solve(start_state, target_pod_location, sensor_data)

    def bfs_solve_path_to_closest_goal(self, sensor_data: dict):
        # BFS to find the shortest path to the closest goal
        start_state = sensor_data[SensorData.PLAYER_LOCATION]
        goals = sensor_data[SensorData.GOAL_LOCATIONS]

        closest_goal = min(goals, key=lambda goal: self.manhattan_distance(start_state, goal))
        self.path = self.bfs_solve(start_state, closest_goal, sensor_data)

    def bfs_solve(self, start, goal, sensor_data):
        # Generic BFS to find path from start to goal
        visited_states = set()
        queue = deque([[DriveState(x=start[0], y=start[1])]])

        while queue:
            current_path = queue.popleft()
            curr_state = current_path[-1]

            if curr_state in visited_states:
                continue

            visited_states.add(curr_state)

            if [curr_state.x, curr_state.y] == goal:
                return current_path

            for state in self.list_all_next_possible_states(curr_state):
                if state not in visited_states and self.is_state_in_bounds(state, sensor_data):
                    queue.append(current_path + [state])

        print('WARN: BFS could not find a path to the goal')
        return []

    def list_all_next_possible_states(self, state: DriveState) -> list[int]:
        # Returns a list of all reachable states from the argument state by iterating over all possible drive moves
        next_states = []
        for move in DriveMove:
            x, y = state.get_next_state_from_move(move)
            next_states.append(DriveState(x=x, y=y))
        return next_states

    def is_state_in_bounds(self, state: DriveState, sensor_data: dict) -> bool:
        # Check if the state is not a field boundary or occupied by another drive
        if [state.x, state.y] in sensor_data[SensorData.FIELD_BOUNDARIES]:
            return False
        return True

    def is_at_target_pod(self, sensor_data: dict) -> bool:
        # Check if the player is at the target pod location
        player_location = sensor_data[SensorData.PLAYER_LOCATION]
        return player_location == sensor_data[SensorData.TARGET_POD_LOCATION]

    def is_at_goal(self, sensor_data: dict) -> bool:
        # Check if the player is at a goal location
        player_location = sensor_data[SensorData.PLAYER_LOCATION]
        return player_location in sensor_data[SensorData.GOAL_LOCATIONS]

    def manhattan_distance(self, start, goal) -> int:
        # Compute the Manhattan distance between two points
        return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

