#!/usr/bin/env python3

# Transform from marker to eye (camera)

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped

def main():
    # Initialize the ROS node
    rospy.init_node('tf_broadcaster_m2e')

    # Create a StaticTransformBroadcaster
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Initialize the transform
    static_transformStamped = TransformStamped()

    # Time and frame information
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "camera_color_frame"
    static_transformStamped.child_frame_id = "marker_frame"

    # Translation vector 
    static_transformStamped.transform.translation.x = 0.097577
    static_transformStamped.transform.translation.y = 0.146278
    static_transformStamped.transform.translation.z = 0.420846

    # Rotation vector converted to quaternion 
    quaternion = tf_conversions.transformations.quaternion_from_euler(-2.20278, 2.19708, 0.0785671)

    static_transformStamped.transform.rotation.x = quaternion[0]
    static_transformStamped.transform.rotation.y = quaternion[1]
    static_transformStamped.transform.rotation.z = quaternion[2]
    static_transformStamped.transform.rotation.w = quaternion[3]

    # Broadcast the transform
    broadcaster.sendTransform(static_transformStamped)

    # Keep the node alive
    rospy.spin()

if __name__ == '__main__':
    main()