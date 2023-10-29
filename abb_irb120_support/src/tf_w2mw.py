#!/usr/bin/env python3

# Transform from world to moving_world

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def main():
    # Initialize the ROS node
    rospy.init_node('tf_broadcaster_world_to_moving_world')

    # Create a StaticTransformBroadcaster
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Initialize the transform
    static_transformStamped = TransformStamped()

    # Time and frame information
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = "moving_world"

    # Translation vector
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.2  

    # Quaternion for rotation (no rotation in this case)
    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0

    # Broadcast the transform
    broadcaster.sendTransform(static_transformStamped)

    # Keep the node alive
    rospy.spin()

if __name__ == '__main__':
    main()