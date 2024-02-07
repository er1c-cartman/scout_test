import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import TransformStamped

class OdomToVLP16TransformPublisher(Node):
    def __init__(self):
        super().__init__('odom_to_vlp16_transform_publisher')
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1/20, self.publish_odom_to_vlp16_transform)

    def publish_odom_to_vlp16_transform(self):
        try:
            # Get the transform from odom to base_link
            trans_odom_base = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())

            # Static transforms from base_link to vlp16
            static_transforms = [
                ('base_link', 'vlp16_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                ('vlp16_link', 'vlp16_base_link', (0.0, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0)),
                ('vlp16_base_link', 'vlp16', (0.0, 0.0, 0.0377), (0.0, 0.0, 0.0, 1.0)),
            ]

            # Calculate the cumulative transform from odom to vlp16
            cumulative_transform = tf_transformations.concatenate_matrices(
                tf_transformations.translation_matrix((trans_odom_base.transform.translation.x, trans_odom_base.transform.translation.y, trans_odom_base.transform.translation.z)),
                tf_transformations.quaternion_matrix((trans_odom_base.transform.rotation.x, trans_odom_base.transform.rotation.y, trans_odom_base.transform.rotation.z, trans_odom_base.transform.rotation.w))
            )

            for parent, child, translation, rotation in static_transforms:
                static_trans = tf_transformations.concatenate_matrices(
                    tf_transformations.translation_matrix(translation),
                    tf_transformations.quaternion_matrix(rotation)
                )
                cumulative_transform = tf_transformations.concatenate_matrices(cumulative_transform, static_trans)

            # Extract translation and rotation from the cumulative transform
            trans = tf_transformations.translation_from_matrix(cumulative_transform)
            rot = tf_transformations.quaternion_from_matrix(cumulative_transform)

            # Create and send the TransformStamped message
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'vlp16'
            t.transform.translation.x = trans[0]
            t.transform.translation.y = trans[1]
            t.transform.translation.z = trans[2]
            t.transform.rotation.x = rot[0]
            t.transform.rotation.y = rot[1]
            t.transform.rotation.z = rot[2]
            t.transform.rotation.w = rot[3]

            self.broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error('Failed to compute or publish transform: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = OdomToVLP16TransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
