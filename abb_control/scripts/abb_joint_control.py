#!/usr/bin/env python3

from numpy import pi
import argparse

import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal

def functional(config):

    name = "dual_arm"
    rospy.init_node(f'{name}_joint_planning')
    
    group= moveit_commander.MoveGroupCommander(f"{name}")
    group.set_start_state_to_current_state()

    pub = rospy.Publisher(f'/{name}/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=1)
    display_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)

    joint_start = group.get_current_joint_values()
    print("Starting joint values: ", joint_start)
    if name == "dual_arm":
        if config == 'all_zero':
            joint_goal = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0, 0.0, 0.0, 0.0]
        elif config == 'initial':
            joint_goal = [3.9335634937742725e-05, 0.8073373436927795, -1.0971498489379883, 4.13949201174546e-05, 0.28975585103034973, 8.134220843203366e-05, 0, 0.0, 0, 0.0, 0.0, 0.0]
    elif name == "mp":
        joint_goal = [0.9268576502799988, 
                      0.2813063859939575, 
                      0.7393782734870911,
                      1.145723581314087, 
                      -1.0036907196044922,
                      -0.9744566082954407]

    print("Goal joint values: ", joint_goal)

    joint_names = ['joint_2', 'joint_3', 'joint_4', 'joint_5', \
                'joint_6', 'joint_7', 'joint_2_m', 'joint_3_m', \
                'joint_4_m', 'joint_5_m', 'joint_6_m', 'joint_7_m']
    
    group.set_joint_value_target(joint_goal)
    _, plan, _, _ = group.plan()
    
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = joint_names
    joint_state.position = joint_start

    display_trajectory = DisplayTrajectory()
    display_trajectory.model_id = 'dual_arm'
    display_trajectory.trajectory.append(plan)
    display_trajectory.trajectory_start.joint_state = joint_state

    message = FollowJointTrajectoryActionGoal()
    message.goal.trajectory = plan

    pub.publish(message)
    display_pub.publish(display_trajectory)
    execution = group.go(wait=True)
    print("Executed? ", execution)
    group.stop()
    group.clear_pose_targets()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Desginate a standard joint configuration for group 'dual_arm'")
    parser.add_argument("config", help="Indicate configuration: 'all_zero' or 'initial'")

    args = parser.parse_args()

    try:
        functional(args.config)
    except rospy.ROSInterruptException:
        pass