#! /usr/bin/env python3

import random
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

destinations = {
    "A": [0.7, 2.0],
    "B": [-0.7, 2.0],
    "C": [1.9, -0.6],
    "D": [-2.2, 0.0],
}

def main():
    rclpy.init()

    robots = ["robot1", "robot2"]

    navigator = Navigator(robots)

    remaining_destinations = list(destinations.keys())
    dests = {}
    for robot in robots:
        dests[robot] = random.choice(remaining_destinations)
        remaining_destinations.remove(dests[robot])
        print(f"[{robot}] Destination: {dests[robot]} (pos: {destinations[dests[robot]]})")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = destinations[dests[robot]][0]
        goal_pose.pose.position.y = destinations[dests[robot]][1]
        goal_pose.pose.orientation.w = 1.0

        navigator.go_to_pose(robot, goal_pose)

    i = 0
    while not all([navigator.is_task_complete(robot) for robot in robots]):
        i = i + 1
        for robot in robots:
            if navigator.feedback[robot] and i % 5 == 0:
                s = Duration.from_msg(navigator.feedback[robot].estimated_time_remaining).nanoseconds / 1e9
                print(f"[{robot}] Estimated time of arrival: "+ "{0:.0f}".format(s) + " seconds.")

    for robot in robots:
        if navigator.status[robot] == GoalStatus.STATUS_SUCCEEDED:
            print(f"[{robot}] Goal succeeded!")
        elif navigator.status[robot] == GoalStatus.STATUS_CANCELED:
            print(f"[{robot}] Goal was canceled!")
        elif navigator.status[robot] == GoalStatus.STATUS_ABORTED:
            print(f"[{robot}] Goal failed!")
        else:
            print(f"[{robot}] Goal has an invalid return status!")


class Navigator(Node):
    def __init__(self, robots, node_name='navigator'):
        super().__init__(node_name=node_name)
        self.feedback = {robot: None for robot in robots}
        self.status = {robot: None for robot in robots}
        self.goal_handle = {robot: None for robot in robots}
        self.result_future = {robot: None for robot in robots}
        self.action_clients = {}
        for robot in robots:
            self.action_clients[robot] = ActionClient(self, NavigateToPose, f'{robot}/navigate_to_pose')

    def go_to_pose(self, robot, pose, behavior_tree=''):
        while not self.action_clients[robot].wait_for_server(timeout_sec=1.0):
            print("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        send_goal_future = self.action_clients[robot].send_goal_async(goal_msg, lambda msg: self._feedback_callback(robot, msg))
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle[robot] = send_goal_future.result()

        if not self.goal_handle[robot].accepted:
            print('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future[robot] = self.goal_handle[robot].get_result_async()
        return True

    def is_task_complete(self, robot):
        if not self.result_future[robot]:
            return True
        rclpy.spin_until_future_complete(self, self.result_future[robot], timeout_sec=0.10)
        if self.result_future[robot].result():
            self.status[robot] = self.result_future[robot].result().status
            if self.status[robot] != GoalStatus.STATUS_SUCCEEDED:
                return True
        else:
            return False

        return True

    def _feedback_callback(self, robot, msg):
        self.feedback[robot] = msg.feedback


if __name__ == "__main__":
    main()
