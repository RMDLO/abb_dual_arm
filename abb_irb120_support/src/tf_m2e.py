#!/usr/bin/env python3

# Transform from marker to eye (camera)

import rospy
import tf2_ros
#import tf_conversions
from geometry_msgs.msg import TransformStamped
import numpy as np
import cv2
import os

def matrix_to_quaternion(R):
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = R.flatten()
    tr = m00 + m11 + m22

    if tr > 0:
        S = np.sqrt(tr+1.0) * 2  # S=4*qw 
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2  # S=4*qx 
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2  # S=4*qy
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2  # S=4*qz
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    return [qx, qy, qz, qw]

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

    clearance_m = 0.01;  # 10 mm in meters

    # Translation vector from file
    static_transformStamped.transform.translation.x = tvec[0]
    static_transformStamped.transform.translation.y = tvec[1]
    static_transformStamped.transform.translation.z = tvec[2];

    # Rotation vector from file converted to quaternion
    R, _ = cv2.Rodrigues(rvec)

    # Create a 4x4 homogeneous transformation matrix
    R_homogeneous = np.eye(4)
    R_homogeneous[:3, :3] = R

    #quaternion = tf_conversions.transformations.quaternion_from_matrix(R_homogeneous)

    # Assuming R_homogeneous is a 4x4 matrix
    R_33 = R_homogeneous[:3, :3]  # Extract the top-left 3x3 rotation matrix
    quaternion = matrix_to_quaternion(R_33)

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