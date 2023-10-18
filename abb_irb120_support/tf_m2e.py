#!/usr/bin/env python3

# Transform from marker to eye (camera)

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
import numpy as np
import cv2

def main():
    # Initialize the ROS node
    rospy.init_node('tf_broadcaster_m2e')

    # Create a StaticTransformBroadcaster
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Initialize the transform
    static_transformStamped = TransformStamped()

    # Time and frame information
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "camera_color_optical_frame"
    static_transformStamped.child_frame_id = "marker_frame"

    # Translation vector
    static_transformStamped.transform.translation.x = 0.109172
    static_transformStamped.transform.translation.y = -0.0076801
    static_transformStamped.transform.translation.z = 0.453516

    # New Rotation vector converted to quaternion
    rvec = np.array([2.01053, -2.01678, 0.389023])
    R, _ = cv2.Rodrigues(rvec)

    # Create a 4x4 homogeneous transformation matrix
    R_homogeneous = np.eye(4)
    R_homogeneous[:3, :3] = R

    quaternion = tf_conversions.transformations.quaternion_from_matrix(R_homogeneous)

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