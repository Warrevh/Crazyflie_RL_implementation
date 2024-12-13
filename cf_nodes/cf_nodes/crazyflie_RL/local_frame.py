import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import numpy as np

class LocalFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('local_frame_broadcaster')

        # Initialize the broadcaster
        self.br = TransformBroadcaster(self)

        # Create a timer to broadcast at a regular interval
        self.timer = self.create_timer(1/480, self.broadcast_transform)

        self.translation = [-2.0, 2.5, 0.0]  # x, y, z
        self.rotation = [0.0, 0.0, -np.pi / 2]  # roll, pitch, yaw (90 degrees around Z-axis)

    def broadcast_transform(self):
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the parent and child frames
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'local_frame'

        # Set translation
        t.transform.translation.x = self.translation[0]
        t.transform.translation.y = self.translation[1]
        t.transform.translation.z = self.translation[2]

        # Set rotation (quaternion)
        quaternion = tf_transformations.quaternion_from_euler(self.rotation[0], self.rotation[1], self.rotation[2])
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Broadcast the transformation
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LocalFrameBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()