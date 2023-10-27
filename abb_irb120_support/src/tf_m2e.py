#!/usr/bin/env python3

# Transform from marker to eye (camera)

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
import numpy as np
import cv2
import os

def read_rvec_tvec_from_file(filename):
    with open(filename, 'r') as file:
        try:
            line = file.readline().strip()
            rvec = np.array([float(x) for x in line.split('[')[1].split(']')[0].split(',')])

            line = file.readline().strip()
            tvec = np.array([float(x) for x in line.split('[')[1].split(']')[0].split(',')])

            return rvec, tvec

        except Exception as e:
            print(f"An error occurred: {e}")
            return None, None

def main():
    # Initialize the ROS node
    rospy.init_node('tf_broadcaster_m2e')

   # Get the directory in which this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Construct the full path to the text file
    text_file_path = os.path.join(script_dir, "charuco_corner_rvec_tvec.txt")

    # Read rvec and tvec from the text file
    rvec, tvec = read_rvec_tvec_from_file(text_file_path)

    if rvec is None or tvec is None:
        print("Could not read rvec or tvec from file. Exiting.")
        return

    # Create a StaticTransformBroadcaster
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Initialize the transform
    static_transformStamped = TransformStamped()

    # Time and frame information
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "camera_color_optical_frame"
    static_transformStamped.child_frame_id = "marker_frame"

    # Translation vector from file
    static_transformStamped.transform.translation.x = tvec[0]
    static_transformStamped.transform.translation.y = tvec[1]
    static_transformStamped.transform.translation.z = tvec[2]

    # Rotation vector from file converted to quaternion
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