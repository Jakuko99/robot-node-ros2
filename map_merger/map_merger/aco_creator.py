class ACOCreator:
    def __init__(self, robot_name: str, parent) -> None:
        self.robot_name = robot_name
        self.parent = parent

    def update_transform(self, mgs):
        pass

    def update_local_map(self, msg):
        pass

    def update_global_map(self, msg):
        pass

    def update_odom(self, msg):
        pass
