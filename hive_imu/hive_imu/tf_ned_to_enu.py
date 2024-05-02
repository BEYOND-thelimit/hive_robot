import rclpy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

def publish_tf():
    rclpy.init()
    node = rclpy.create_node('tf_publisher')

    tf_broadcaster = tf2_ros.StaticTransformBroadcaster(node)

    # Create a TransformStamped message
    static_transformStamped = TransformStamped()
    static_transformStamped.header.frame_id = 'imu_link_ned'
    static_transformStamped.child_frame_id = 'imu_link_enu'

    # Define the transformation (rotation and translation)
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0
    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = math.sin(math.pi / 2)  # Example rotation
    static_transformStamped.transform.rotation.w = math.cos(math.pi / 2)  # Example rotation

    # Publish the TF message
    tf_broadcaster.sendTransform(static_transformStamped)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    publish_tf()
