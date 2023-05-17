#!/usr/bin/env python3

import rospy
import moveit_commander
from std_msgs.msg import Header
from control_msgs.msg import FollowJointTrajectoryActionGoal
from numpy import pi

def functional():

    rospy.init_node('dual_arm_planning')

    # Initialize the robot commander for each robot arm
    robot1 = moveit_commander.RobotCommander(ns="mp")
    robot2 = moveit_commander.RobotCommander(ns="mp_m")
    scene = moveit_commander.PlanningSceneInterface()
    
    # Create a move group commander for each robot arm
    group1 = moveit_commander.MoveGroupCommander("mp")
    group2 = moveit_commander.MoveGroupCommander("mp_m")
    group1.set_start_state_to_current_state()
    group2.set_start_state_to_current_state()

    pub_rob1 = rospy.Publisher('/mp/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=1)
    pub_rob2 = rospy.Publisher('/mp_m/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=1)

    joint_goal1 = group1.get_current_joint_values()
    joint_goal2 = group2.get_current_joint_values()

    # Set the joint goal for each robot arm
    joint_goal1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Joint values for arm1
    joint_goal2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Joint values for arm2

    message_rob1 = FollowJointTrajectoryActionGoal()
    message_rob2 = FollowJointTrajectoryActionGoal()

    joint_names = ['joint_2', 'joint_3', 'joint_4', 'joint_5', \
                  'joint_6', 'joint_7']
    joint_names_m = ['joint_2_m', 'joint_3_m', 'joint_4_m', 'joint_5_m', \
                  'joint_6_m', 'joint_7_m']
    message_rob1.goal.trajectory.joint_names = joint_names
    message_rob2.goal.trajectory.joint_names = joint_names_m

    group1.set_joint_value_target(joint_goal1)
    group2.set_joint_value_target(joint_goal2)

    plan1 = group1.plan()
    plan2 = group2.plan()

    traj1 = plan1[1]
    traj2 = plan2[1]

    message_rob1.goal.trajectory = traj1.joint_trajectory
    message_rob2.goal.trajectory = traj2.joint_trajectory

    header_rob1 = Header()
    header_rob1.stamp = rospy.Time.now()
    message_rob1.goal.trajectory.header = header_rob1

    header_rob2 = Header()
    header_rob2.stamp = rospy.Time.now()
    message_rob2.goal.trajectory.header = header_rob2

    #  publishing to ROS node
    pub_rob1.publish(message_rob1)
    pub_rob2.publish(message_rob2)


if __name__ == '__main__':
    try:
        functional()
    except rospy.ROSInterruptException:
        pass