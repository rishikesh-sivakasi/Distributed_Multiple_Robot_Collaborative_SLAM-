import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import numpy as np
import math


class MultiRobotExplorer(Node):

    def __init__(self):
        super().__init__('multi_robot_explorer')

        # -------------------- ROBOT CONFIG --------------------
        self.robots = {
            "burger": {
                "map": "burger/map",
                "base": "burger/base_footprint",
                "action": "burger/navigate_to_pose",
                "busy": False,
                "target": None
            },
            "waffle_pi": {
                "map": "waffle_pi/map",
                "base": "waffle_pi/base_footprint",
                "action": "waffle_pi/navigate_to_pose",
                "busy": False,
                "target": None
            }
        }

        # -------------------- TF --------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------------------- NAV2 ACTION CLIENTS --------------------
        self.nav_clients = {}
        for r in self.robots:
            self.nav_clients[r] = ActionClient(
                self, NavigateToPose, self.robots[r]["action"]
            )

        # -------------------- MAP --------------------
        self.map = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        # Failed goals blacklist
        self.blacklist = []

        self.get_logger().info("Multi-Robot Explorer started")

    # ============================================================
    # MAP CALLBACK
    # ============================================================
    def map_callback(self, msg):
        self.map = msg
        self.assign_goals()

    # ============================================================
    # ASSIGN GOALS
    # ============================================================
    def assign_goals(self):
        if self.map is None:
            return

        frontiers = self.find_frontiers(self.map)
        if not frontiers:
            self.get_logger().info("Exploration complete")
            return

        for robot in self.robots:
            if not self.robots[robot]["busy"]:
                target = self.select_frontier(robot, frontiers)
                if target:
                    self.send_goal(robot, target)

    # ============================================================
    # FRONTIER DETECTION
    # ============================================================
    def find_frontiers(self, map_msg):
        w, h = map_msg.info.width, map_msg.info.height
        data = np.array(map_msg.data).reshape((h, w))
        frontiers = []

        for y in range(2, h - 2):
            for x in range(2, w - 2):
                if data[y, x] == 0:
                    window = data[y-1:y+2, x-1:x+2]
                    if -1 in window:
                        frontiers.append((x, y))

        return frontiers

    # ============================================================
    # SELECT BEST FRONTIER
    # ============================================================
    def select_frontier(self, robot, frontiers):
        pose = self.get_robot_pose(robot)
        if pose is None:
            return None

        rx, ry = pose
        best = None
        best_dist = float("inf")

        for cell in frontiers:
            wx, wy = self.cell_to_world(cell)

            # Skip very close goals
            dist = math.hypot(wx - rx, wy - ry)
            if dist < 1.5:
                continue

            # Skip blacklisted goals
            if any(math.hypot(wx - b[0], wy - b[1]) < 1.0 for b in self.blacklist):
                continue

            # Keep robots separated
            for other in self.robots:
                if self.robots[other]["target"]:
                    ox, oy = self.robots[other]["target"]
                    if math.hypot(wx - ox, wy - oy) < 3.0:
                        continue

            if dist < best_dist:
                best_dist = dist
                best = (wx, wy)

        return best

    # ============================================================
    # TF LOOKUP
    # ============================================================
    def get_robot_pose(self, robot):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.robots[robot]["map"],
                self.robots[robot]["base"],
                rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return x, y

        except Exception as e:
            self.get_logger().warn(f"{robot} TF error: {e}")
            return None

    # ============================================================
    # SEND NAV2 GOAL
    # ============================================================
    def send_goal(self, robot, target):
        client = self.nav_clients[robot]

        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"{robot}: Nav2 not available")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.robots[robot]["map"]
        goal.pose.pose.position.x = target[0]
        goal.pose.pose.position.y = target[1]
        goal.pose.pose.orientation.w = 1.0

        self.robots[robot]["busy"] = True
        self.robots[robot]["target"] = target

        self.get_logger().info(
            f" {robot} navigating to ({target[0]:.2f}, {target[1]:.2f})"
        )

        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, r=robot: self.goal_response(f, r)
        )

    # ============================================================
    # GOAL RESPONSE
    # ============================================================
    def goal_response(self, future, robot):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(f"{robot} goal rejected")
            self.handle_failure(robot)
            return

        goal_handle.get_result_async().add_done_callback(
            lambda f, r=robot: self.goal_result(f, r)
        )

    def goal_result(self, future, robot):
        status = future.result().status

        if status == 4:
            self.get_logger().info(f"{robot} reached goal")
        else:
            self.get_logger().warn(f"{robot} failed, blacklisting")
            self.blacklist.append(self.robots[robot]["target"])

        self.robots[robot]["busy"] = False
        self.robots[robot]["target"] = None
        self.assign_goals()

    # ============================================================
    # UTIL
    # ============================================================
    def cell_to_world(self, cell):
        res = self.map.info.resolution
        origin = self.map.info.origin.position
        return (
            origin.x + (cell[0] + 0.5) * res,
            origin.y + (cell[1] + 0.5) * res
        )

    def handle_failure(self, robot):
        if self.robots[robot]["target"]:
            self.blacklist.append(self.robots[robot]["target"])
        self.robots[robot]["busy"] = False
        self.robots[robot]["target"] = None


def main():
    rclpy.init()
    node = MultiRobotExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
