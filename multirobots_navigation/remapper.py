#!/usr/bin/env python3
print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage


from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class TfRemapper(Node):

    def __init__(self):
        super().__init__('tf_remapper')
        self.subscription1 = self.create_subscription(TFMessage, '/robot1/tf', self.robot1_callback, 10)
        self.subscription2 = self.create_subscription(TFMessage, '/robot2/tf', self.robot2_callback, 10)
        # self.subscription1_static = self.create_subscription(TFMessage, '/robot1/tf_static', self.robot1_callback_static, 10)
        # self.subscription2_static = self.create_subscription(TFMessage, '/robot2/tf_static', self.robot2_callback_static, 10)
        self.publisher_tf = self.create_publisher(TFMessage, '/tf', 10)
        # self.publisher_tf_static = StaticTransformBroadcaster(self)

    def robot1_callback(self, msg: TFMessage):
        for t in msg.transforms:
            t.header.frame_id = "/robot1/" + t.header.frame_id
            t.child_frame_id = "/robot1/" + t.child_frame_id
        # print("robot1_callback", msg)
        self.publisher_tf.publish(msg)

    def robot2_callback(self, msg: TFMessage):
        for t in msg.transforms:
            t.header.frame_id = "/robot2/" + t.header.frame_id
            t.child_frame_id = "/robot2/" + t.child_frame_id
        # print("robot2_callback", msg)
        self.publisher_tf.publish(msg)

    def robot1_callback_static(self, msg: TFMessage):
        print("robot1_callback_static", msg)
        for t in msg.transforms:
            t.header.frame_id = "/robot1" + t.header.frame_id
            t.child_frame_id = "/robot1" + t.child_frame_id
            self.publisher_tf_static.sendTransform(t)

    def robot2_callback_static(self, msg: TFMessage):
        print("robot2_callback_static", msg)
        for t in msg.transforms:
            t.header.frame_id = "/robot2" + t.header.frame_id
            t.child_frame_id = "/robot2" + t.child_frame_id
            self.publisher_tf_static.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = TfRemapper()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
