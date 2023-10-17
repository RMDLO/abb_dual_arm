#!/usr/bin/env python3

# Transform from hand to eye (end effector to camera)

import rospy
import tf
from tf.transformations import quaternion_from_matrix
import numpy as np

def broadcast_tf():
    rospy.init_node('tf_broadcaster_h2e')

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10)  # 10Hz

    # Your transformation matrix
    matrix = np.array([
        [0.0047183,   -0.999864,  -0.0158078,   0.0179451],
        [-0.999987, -0.00474907,   0.0019095,    0.030616],
        [-0.00198431,   0.0157986,   -0.999873,   -0.039238],
        [0,           0,           0,           1]
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
            "eye",  # Child frame (object in this case)
            "link_6"  # Parent frame (world in this case)
        )
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_tf()
    except rospy.ROSInterruptException:
        pass