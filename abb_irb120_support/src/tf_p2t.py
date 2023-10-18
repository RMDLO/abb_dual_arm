#!/usr/bin/env python3

# Static transformation from pointer to pointer's tip

import rospy
import tf
from tf.transformations import quaternion_from_matrix
import numpy as np

def broadcast_tf():
    rospy.init_node('tf_broadcaster_p2t')

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10)  # 10Hz

    # Your transformation matrix
    matrix = np.array([
        [1, 0, 0, -0.03],
        [0, 1, 0, 0],
        [0, 0, 1, -0.06],
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
            "tip",  # Child frame (object in this case)
            "pointer_m"  # Parent frame (world in this case)
        )
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_tf()
    except rospy.ROSInterruptException:
        pass