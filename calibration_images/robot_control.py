class RobotControl:
    def __init__(self, initial_position):
        self.current_position = initial_position

    def get_current_position(self):
        return self.current_position

    def set_current_position(self, new_position):
        self.current_position = new_position

    def move_to_position(self, target_position):
        self.current_position = target_position
        print(f"Robot moved to position: {target_position}")
