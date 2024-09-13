from src.DriveInterface import DriveInterface
from src.DriveState import DriveState
from src.Constants import DriveMove, SensorData
from collections import deque

class SameerAgent(DriveInterface):

    def __init__(self, game_id: int, is_advanced_mode: bool):
        # Constructor for player
        self.game_id = game_id
        self.path = []
        self.path_move_index = 0
        self.need_to_find_target_pod = is_advanced_mode

    def get_next_move(self, sensor_data: dict) -> DriveMove:
        # Main function called by game orchestrator
        if len(self.path) == 0:
            if self.need_to_find_target_pod:
                raise Exception('Advanced mode solver not implemented yet for SameerAgent')
            else:
                # Use BFS to find the closest goal
                self.bfs_solve_path_to_closest_goal(sensor_data)

        next_move, next_state = self.get_move_for_next_state_in_path()
        if self.will_next_state_collide(next_state, sensor_data):
            self.path_move_index -= 1
            return DriveMove.NONE
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
        # Function to find the move which will get the player to the next state in self.path
        current_state = self.path[self.path_move_index]
        self.path_move_index += 1
        next_state = self.path[self.path_move_index]

        for move in DriveMove:
            if current_state.get_next_state_from_move(move) == next_state.to_tuple():
                return move, next_state

        print('WARN: Next move could not be found')
        return DriveMove.NONE, next_state

    def bfs_solve_path_to_closest_goal(self, sensor_data: dict):
        # Breadth First Search (BFS) solver to find the shortest path to the closest goal
        start_state = sensor_data[SensorData.PLAYER_LOCATION]
        goals = sensor_data[SensorData.GOAL_LOCATIONS]
        field_boundaries = sensor_data[SensorData.FIELD_BOUNDARIES]
        other_drives = sensor_data[SensorData.DRIVE_LOCATIONS]

        visited_states = set()
        queue = deque([[DriveState(x=start_state[0], y=start_state[1])]])
        shortest_path = None

        while queue:
            current_path = queue.popleft()
            curr_state = current_path[-1]

            if curr_state in visited_states:
                continue

            visited_states.add(curr_state)

            # If we reached any goal, we're done (since BFS guarantees shortest path)
            if [curr_state.x, curr_state.y] in goals:
                shortest_path = current_path
                break

            # Explore all possible next states (valid moves)
            for state in self.list_all_next_possible_states(curr_state):
                if state not in visited_states and self.is_state_in_bounds(state, sensor_data) and not self.is_location_occupied(state, field_boundaries, other_drives):
                    queue.append(current_path + [state])

        if shortest_path:
            self.path = shortest_path
        else:
            print('WARN: BFS could not find a path to any goal')

    def list_all_next_possible_states(self, state: DriveState) -> list[int]:
        # Returns a list of all reachable states from the argument state by iterating over all possible drive moves
        next_states = []
        for move in DriveMove:
            x, y = state.get_next_state_from_move(move)
            next_states.append(DriveState(x=x, y=y))
        return next_states

    def is_state_in_bounds(self, state: DriveState, sensor_data: dict) -> bool:
        # Checks if state argument is not a field wall or out of bounds
        return [state.x, state.y] not in sensor_data[SensorData.FIELD_BOUNDARIES]

    def is_location_occupied(self, state: DriveState, field_boundaries: list, other_drives: list) -> bool:
        # Check if the state is occupied by a wall or another drive
        if [state.x, state.y] in field_boundaries or [state.x, state.y] in other_drives:
            return True
        return False

    def is_player_drive_carrying_a_pod(self, sensor_data: dict) -> bool:
        # Checks if player game id is the first value in any of the entries in SensorData.DRIVE_LIFTED_POD_PAIRS
        return self.game_id in [drive_pod_pair[0] for drive_pod_pair in sensor_data[SensorData.DRIVE_LIFTED_POD_PAIRS]]
