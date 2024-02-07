#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transforms)

    def publish_transforms(self):
        current_time = self.get_clock().now().to_msg()

        # List of transforms to be published
        transforms = [
        
            {
                "parent_frame": "base_link",
                "child_frame": "base_footprint",
                "translation": [0.0, 0.0, -0.23479],
                "rotation": [0.0, 0.0, 0.0, 1.0]
            },
            {
                "parent_frame": "base_link",
                "child_frame": "imu_link",
                "translation": [0.19, 0.0, 0.149],
                "rotation": [0.0, 0.0, 0.0, 1.0]
            },
            {
                "parent_frame": "base_link",
                "child_frame": "inertial_link",
                "translation": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0, 1.0]
            },
            {
                "parent_frame": "base_laser_mount",
                "child_frame": "base_laser",
                "translation": [0.0352, 0.0, 0.0566],
                "rotation": [0.9999999999991198, 0.0, 0.0, 1.3267948966775328e-06]
            },
            {
                "parent_frame": "base_link",
                "child_frame": "base_laser_mount",
                "translation": [0.2, 0.0, 0.4],
                "rotation": [0.0, 0.0, 0.0, 1.0]
            }
        ]

        for transform in transforms:
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = transform["parent_frame"]
            t.child_frame_id = transform["child_frame"]
            t.transform.translation.x = transform["translation"][0]
            t.transform.translation.y = transform["translation"][1]
            t.transform.translation.z = transform["translation"][2]
            t.transform.rotation.x = transform["rotation"][0]
            t.transform.rotation.y = transform["rotation"][1]
            t.transform.rotation.z = transform["rotation"][2]
            t.transform.rotation.w = transform["rotation"][3]
            self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
