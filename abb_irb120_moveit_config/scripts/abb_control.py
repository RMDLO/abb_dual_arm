#!/usr/bin/env python3

import rospy
import moveit_commander
from std_msgs.msg import Header
from moveit_msgs.msg import DisplayTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from numpy import pi

def functional():

    rospy.init_node('dual_arm_planning')

    # Create a move group commander
    group= moveit_commander.MoveGroupCommander("dual_arm")
    group.set_start_state_to_current_state()

    pub = rospy.Publisher('/dual_arm/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=1)
    display_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)

    joint_goal = group.get_current_joint_values()

    message = FollowJointTrajectoryActionGoal()
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = (joint_goal)

    # Set the joint goal for each robot arm
    joint_goal = [pi/4, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, pi/4, 0.0, 0.0, 0.0]
    joints = ['joint_2', 'joint_3', 'joint_4', 'joint_5', \
                'joint_6', 'joint_7', 'joint_2_m', 'joint_3_m', \
                'joint_4_m', 'joint_5_m', 'joint_6_m', 'joint_7_m']

    message.goal.trajectory.joint_names = joints
    display_trajectory.model_id = 'dual_arm'

    group.set_joint_value_target(joint_goal)

    plan = group.plan()

    traj = plan[1]

    message.goal.trajectory = traj.joint_trajectory
    display_trajectory.trajectory = traj.joint_trajectory

    header = Header()
    header.stamp = rospy.Time.now()
    message.goal.trajectory.header = header

    #  publishing to ROS node
    pub.publish(message)
    display_pub.publish(display_trajectory)
    execution = group.go(wait=True)
    print(execution)
    group.stop()
    group.clear_pose_targets()

if __name__ == '__main__':
    try:
        functional()
    except rospy.ROSInterruptException:
        pass