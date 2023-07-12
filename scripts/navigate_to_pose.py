# #! /usr/bin/env python3
# # Copyright 2021 Samsung Research America
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

# from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# import rclpy
# from rclpy.duration import Duration

# """
# Basic navigation demo to go to pose.
# """


# def main():
#     rclpy.init()

#     navigator = BasicNavigator()

#     # Set our demo's initial pose
#     initial_pose = PoseStamped()
#     initial_pose.header.frame_id = "map"
#     initial_pose.header.stamp = navigator.get_clock().now().to_msg()
#     initial_pose.pose.position.x = 3.45
#     initial_pose.pose.position.y = 2.15
#     initial_pose.pose.orientation.z = 1.0
#     initial_pose.pose.orientation.w = 0.0
#     navigator.setInitialPose(initial_pose)

#     # Activate navigation, if not autostarted. This should be called after setInitialPose()
#     # or this will initialize at the origin of the map and update the costmap with bogus readings.
#     # If autostart, you should `waitUntilNav2Active()` instead.
#     # navigator.lifecycleStartup()

#     # Wait for navigation to fully activate, since autostarting nav2
#     navigator.waitUntilNav2Active()

#     # If desired, you can change or load the map as well
#     # navigator.changeMap('/path/to/map.yaml')

#     # You may use the navigator to clear or obtain costmaps
#     # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
#     # global_costmap = navigator.getGlobalCostmap()
#     # local_costmap = navigator.getLocalCostmap()

#     # Go to our demos first goal pose
#     goal_pose = PoseStamped()
#     goal_pose.header.frame_id = "map"
#     goal_pose.header.stamp = navigator.get_clock().now().to_msg()
#     goal_pose.pose.position.x = -2.0
#     goal_pose.pose.position.y = -0.5
#     goal_pose.pose.orientation.w = 1.0

#     # sanity check a valid path exists
#     # path = navigator.getPath(initial_pose, goal_pose)

#     navigator.goToPose(goal_pose)

#     i = 0
#     while not navigator.isTaskComplete():
#         ################################################
#         #
#         # Implement some code here for your application!
#         #
#         ################################################

#         # Do something with the feedback
#         i = i + 1
#         feedback = navigator.getFeedback()
#         if feedback and i % 5 == 0:
#             print(
#                 "Estimated time of arrival: "
#                 + "{0:.0f}".format(
#                     Duration.from_msg(
#                         feedback.estimated_time_remaining
#                     ).nanoseconds
#                     / 1e9
#                 )
#                 + " seconds."
#             )

#             # Some navigation timeout to demo cancellation
#             if Duration.from_msg(feedback.navigation_time) > Duration(
#                 seconds=600.0
#             ):
#                 navigator.cancelTask()

#             # Some navigation request change to demo preemption
#             if Duration.from_msg(feedback.navigation_time) > Duration(
#                 seconds=18.0
#             ):
#                 goal_pose.pose.position.x = -3.0
#                 navigator.goToPose(goal_pose)

#     # Do something depending on the return code
#     result = navigator.getResult()
#     if result == TaskResult.SUCCEEDED:
#         print("Goal succeeded!")
#     elif result == TaskResult.CANCELED:
#         print("Goal was canceled!")
#     elif result == TaskResult.FAILED:
#         print("Goal failed!")
#     else:
#         print("Goal has an invalid return status!")

#     navigator.lifecycleShutdown()

#     exit(0)


# if __name__ == "__main__":
#     main()


#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Shelf positions for picking
shelf_positions = {
    "from_A": [0, 0.5],
    "from_B": [0, 0.5],
    "from_C": [0, 0.5],
    "from_D": [0, 0.5],
}

# Shipping destination for picked products
shipping_destinations = {
    "to_A": [1, 0.5],
    "to_B": [1, 0.5],
    "to_C": [1, 0.5],
    "to_D": [1, 0.5],
}

"""
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
"""


def main():
    # Recieved virtual request for picking item at Shelf A and bringing to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the shelf ID ("shelf_A") and shipping destination ("pallet_jack7")
    ####################
    request_item_location = "from_C"
    request_destination = "to_A"
    ####################

    print("========================Before Init=======================")

    rclpy.init()

    print("========================After Init=======================")

    navigator = BasicNavigator()

    print("========================Basic Nav=======================")

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.5
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    print("========================before set=======================")
    navigator.setInitialPose(initial_pose)
    print("========================after set + waiting=======================")

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    print("========================after waiting=======================")

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = "map"
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = 1.0
    shelf_item_pose.pose.orientation.w = 0.0

    print(
        "Received request for item picking at " + request_item_location + "."
    )
    navigator.goToPose(shelf_item_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                "Estimated time of arrival at "
                + request_item_location
                + " for worker: "
                + "{0:.0f}".format(
                    Duration.from_msg(
                        feedback.estimated_time_remaining
                    ).nanoseconds
                    / 1e9
                )
                + " seconds."
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(
            "Got product from "
            + request_item_location
            + "! Bringing product to shipping destination ("
            + request_destination
            + ")..."
        )
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = "map"
        shipping_destination.header.stamp = (
            navigator.get_clock().now().to_msg()
        )
        shipping_destination.pose.position.x = shipping_destinations[
            request_destination
        ][0]
        shipping_destination.pose.position.y = shipping_destinations[
            request_destination
        ][1]
        shipping_destination.pose.orientation.z = 1.0
        shipping_destination.pose.orientation.w = 0.0
        navigator.goToPose(shipping_destination)

    elif result == TaskResult.CANCELED:
        print(
            "Task at "
            + request_item_location
            + " was canceled. Returning to staging point..."
        )
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print("Task at " + request_item_location + " failed!")
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == "__main__":
    main()
