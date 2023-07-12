#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class PoseInitializerNode(Node):
    def __init__(self):
        super().__init__("pose_initializer_node")
        rclpy.spin_once(self, timeout_sec=15.0)

        self.initial_pose_received1 = False
        self.initial_pose_received2 = False
        self.publisher_robot1 = self.create_publisher(
            PoseWithCovarianceStamped, "/robot1/initialpose", 10
        )
        self.model_pose_sub1 = self.create_subscription(PoseWithCovarianceStamped,
                                                       '/robot1/amcl_pose',
                                                       self._amclPoseCallback1,
                                                       QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1))

        self.publisher_robot2 = self.create_publisher(
            PoseWithCovarianceStamped, "/robot2/initialpose", 10
        )
        self.model_pose_sub2 = self.create_subscription(PoseWithCovarianceStamped,
                                                       '/robot2/amcl_pose',
                                                       self._amclPoseCallback2,
                                                       QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1))

        self._waitForInitialPose()
        self.get_logger().info("Positions initialized")
        exit(0)

    def publish_message(self, robot):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.z = 0.01
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891909122467

        if robot == 1:
            # Fill in the message fields with the desired values for robot 1
            msg.pose.pose.position.y = 0.5

            self.publisher_robot1.publish(msg)
            self.get_logger().info("Published message to /robot1/initialpose")
        elif robot == 2:
            # Fill in the message fields with the desired values for robot 2
            msg.pose.pose.position.y = -0.5

            self.publisher_robot2.publish(msg)
            self.get_logger().info("Published message to /robot2/initialpose")
    
    def _amclPoseCallback1(self, msg):
        self.get_logger().info("Received callback from robot1")
        self.initial_pose_received1 = True
    
    def _amclPoseCallback2(self, msg):
        self.get_logger().info("Received callback from robot2")
        self.initial_pose_received2 = True

    def _waitForInitialPose(self):
        while not (self.initial_pose_received1 and self.initial_pose_received2):
            self.get_logger().info('Setting initial pose')
            if not self.initial_pose_received1:
                self.publish_message(1)
            if not self.initial_pose_received2:
                self.publish_message(2)
            self.get_logger().info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return


def main(args=None):
    rclpy.init(args=args)
    pose_initializer_node = PoseInitializerNode()
    rclpy.spin(pose_initializer_node)

    pose_initializer_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
