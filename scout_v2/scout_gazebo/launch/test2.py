#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage

class StaticToDynamicTfPublisher(Node):
    def __init__(self):
        super().__init__('static_to_dynamic_tf_publisher')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.handle_tf_static_message,
            10)
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.broadcast_static_transforms)
        self.static_transforms = []

    def handle_tf_static_message(self, msg):
        self.static_transforms.extend(msg.transforms)

    def broadcast_static_transforms(self):
        if self.static_transforms:
            tf_message = TFMessage(transforms=self.static_transforms)
            self.broadcaster.sendTransform(tf_message)

def main(args=None):
    rclpy.init(args=args)
    node = StaticToDynamicTfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

