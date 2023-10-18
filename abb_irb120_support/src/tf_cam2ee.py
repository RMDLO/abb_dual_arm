#!/usr/bin/env python3

# Transform from camera to end effector

import rospy
import tf2_ros
import tf.transformations as tf_trans
from geometry_msgs.msg import TransformStamped

def main():
    rospy.init_node('tf_broadcaster_cam2ee')

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "link_6"
    static_transformStamped.child_frame_id = "camera_link"

    # Translation vector
    static_transformStamped.transform.translation.x = 0.0179451
    static_transformStamped.transform.translation.y = 0.030616
    static_transformStamped.transform.translation.z = -0.039238

    # Rotation matrix
    rotation_matrix = [
        0.0047183, -0.999864, -0.0158078,
        -0.999987, -0.00474907, 0.0019095,
        -0.00198431, 0.0157986, -0.999873
    ]
    
    # Convert the rotation matrix to a quaternion
    quat = tf_trans.quaternion_from_matrix([
        [rotation_matrix[0], rotation_matrix[1], rotation_matrix[2], 0],
        [rotation_matrix[3], rotation_matrix[4], rotation_matrix[5], 0],
        [rotation_matrix[6], rotation_matrix[7], rotation_matrix[8], 0],
        [0, 0, 0, 1]
    ])

    # Populate the quaternion values into the message
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    # Publish the static transform
    broadcaster.sendTransform(static_transformStamped)

    # Keep the node alive
    rospy.spin()

if __name__ == '__main__':
    main()