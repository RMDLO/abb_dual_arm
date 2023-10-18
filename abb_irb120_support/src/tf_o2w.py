#!/usr/bin/env python3

import rospy
import tf
from tf.transformations import quaternion_from_matrix
import numpy as np

def broadcast_tf():
    rospy.init_node('tf_broadcaster_o2w')

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10)  # 10Hz

    # Your transformation matrix
    matrix = np.array([
        [-0.999902, -0.0122137, 0.00679748, 0.430938],
        [0.0121794, -0.999913, -0.00506306, 0.105315],
        [0.00685873, -0.00497978, 0.999964, -0.00824589],
        [0, 0, 0, 1]
    ])

    # Extract translation from the matrix
    translation = matrix[:3, 3]

    # Extract quaternion (rotation) from the matrix
    quaternion = quaternion_from_matrix(matrix)

    while not rospy.is_shutdown():
        # Broadcast the transform
        br.sendTransform(
            translation,
            quaternion,
            rospy.Time.now(),
            "object",  # Child frame (object in this case)
            "base_link"  # Parent frame (world in this case)
        )
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_tf()
    except rospy.ROSInterruptException:
        pass