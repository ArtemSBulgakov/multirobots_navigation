from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseInitializerNode(Node):
    def __init__(self):
        super().__init__("pose_initializer_node")

        self.publisher_robot1 = self.create_publisher(
            PoseWithCovarianceStamped, "/robot1/initialpose", 10
        )

        self.publisher_robot2 = self.create_publisher(
            PoseWithCovarianceStamped, "/robot2/initialpose", 10
        )

        self.timer = self.create_timer(1.0, self.publish_message(1))
        self.timer = self.create_timer(1.0, self.publish_message(2))

        self.get_logger().info("Pose Initializer Node initialized")

    def publish_message(self, robot):
        msg = PoseWithCovarianceStamped()

        if robot == 1:
            # Fill in the message fields with the desired values for robot 1
            msg.header.stamp.sec = 0
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = 0
            msg.pose.pose.position.y = 0.5
            msg.pose.pose.position.z = 0.01
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = 1.0
            msg.pose.pose.orientation.w = 0.0

            self.publisher.publish(msg)
            self.get_logger().info("Published message to /robot1/initialpose")
        elif robot == 2:
            # Fill in the message fields with the desired values for robot 2
            msg.header.stamp.sec = 0
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = 0
            msg.pose.pose.position.y = -0.5
            msg.pose.pose.position.z = 0.01
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = 1.0
            msg.pose.pose.orientation.w = 0.0

            self.publisher.publish(msg)
            self.get_logger().info("Published message to /robot2/initialpose")


def main(args=None):
    rclpy.init(args=args)
    pose_initializer_node = PoseInitializerNode()
    rclpy.spin(pose_initializer_node)

    pose_initializer_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
