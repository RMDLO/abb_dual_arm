#!/usr/bin/env python3

import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose

def functional():
    robot = moveit_commander.RobotCommander()
    group_name = "mp_m"  # or "dual_arm" if that is the intended group
    group = robot.get_group(group_name)
    rospy.init_node(f'{group_name}_cartesian_planning')
    group.set_start_state_to_current_state()

    display_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)

    # Note: group.get_current_pose().pose implementation is buggy! Have to call it so that
    # See here: https://github.com/moveit/moveit/issues/2715
    start_pose = group.get_current_pose().pose
    waypoints = []

    pose_goal = Pose()
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 1
    pose_goal.position.x = 0.2
    pose_goal.position.y = 0
    pose_goal.position.z = 0.3

    waypoints.append(pose_goal)
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0, avoid_collisions=True)

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
    joint_state.position = group.get_current_joint_values()

    display_trajectory = DisplayTrajectory()
    display_trajectory.model_id = group_name
    display_trajectory.trajectory_start.joint_state = joint_state
    display_trajectory.trajectory.append(plan)

    display_pub.publish(display_trajectory)
    
    command_input = input("Execute plan? y or n: ")

    if command_input == "y":
        success = group.execute(plan, wait=True)
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