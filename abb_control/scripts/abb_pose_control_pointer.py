#!/usr/bin/env python3

# Cartesian planning for the ABB IRB120 robot

import rospy
import moveit_commander
import numpy as np
import tf.transformations as tf_transform
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Pose, PoseStamped, Quaternion

def functional():
    group_name = "mp_m"
    rospy.init_node(f'{group_name}_cartesian_planning')
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_start_state_to_current_state()

    pub = rospy.Publisher(f'/{group_name}/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=1)
    display_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)

    # Initialize tf2 Buffer and TransformListener
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)

    # Allow some time for the tf_tree to populate
    rospy.sleep(1.0)

    joint_start = group.get_current_joint_values()
    waypoints = []
    waypoints.append(group.get_current_pose().pose)

    # Retrieve the transform from "tip" to "marker_frame"
    trans = tf_buffer.lookup_transform("pointer_m_tip", "marker_frame", rospy.Time(0), rospy.Duration(1.0))

    # Given 3D points and quaternion for the desired position in marker_frame
    x, y, z = -0.280, -0.424, -0.130  
    quaternion = [-0.649, 0.495, 0.273, 0.509]

    # Create Pose message for the marker in marker_frame
    marker_pose_table = Pose()
    marker_pose_table.position.x = x  
    marker_pose_table.position.y = y  
    marker_pose_table.position.z = z 
    marker_pose_table.orientation.x, marker_pose_table.orientation.y, marker_pose_table.orientation.z, marker_pose_table.orientation.w = quaternion  # Replace with the orientation quaternion of the marker in marker_frame

    marker_pose_stamped = PoseStamped()
    marker_pose_stamped.header.frame_id = "marker_frame"
    marker_pose_stamped.pose = marker_pose_table

    # Transform the marker pose to the robot's tip frame
    marker_pose_tip_stamped = tf2_geometry_msgs.do_transform_pose(marker_pose_stamped, trans)
    marker_pose_tip = marker_pose_tip_stamped.pose

    waypoints.append(marker_pose_tip)
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0, avoid_collisions=True)

    # Joint names
    if group_name == "dual_arm":
        joint_names = ['joint_2', 'joint_3', 'joint_4', 'joint_5', \
                    'joint_6', 'joint_7', 'joint_2_m', 'joint_3_m', \
                    'joint_4_m', 'joint_5_m', 'joint_6_m', 'joint_7_m']
    elif group_name == "mp_m":
        joint_names = ['joint_2_m', 'joint_3_m', 'joint_4_m', \
                       'joint_5_m', 'joint_6_m', 'joint_7_m']
    elif group_name == "mp":
        joint_names = ['joint_2', 'joint_3', 'joint_4', \
                'joint_5', 'joint_6', 'joint_7']

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = joint_names
    joint_state.position = joint_start

    display_trajectory = DisplayTrajectory()
    display_trajectory.model_id = group_name
    display_trajectory.trajectory.append(plan)
    display_trajectory.trajectory_start.joint_state = joint_state

    message = FollowJointTrajectoryActionGoal()
    message.goal.trajectory = plan

    pub.publish(message)
    display_pub.publish(display_trajectory)
    
    command_input = input("Execute plan? y or n: ")

    if command_input == "y":
        group.set_pose_target(marker_pose_tip)  
        success = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        print("Executed? ", success)
    else:
        print("Trajectory execution on robot aborted.")


if __name__ == '__main__':
    try:
        functional()
    except rospy.ROSInterruptException:
        pass