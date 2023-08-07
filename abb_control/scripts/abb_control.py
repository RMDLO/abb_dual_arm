#!/usr/bin/env python3

from numpy import pi

import rospy
import moveit_commander
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal

def functional():

    rospy.init_node('dual_arm_planning')
    
    group= moveit_commander.MoveGroupCommander("dual_arm")
    group.set_start_state_to_current_state()

    pub = rospy.Publisher('/dual_arm/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=1)
    display_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)

    joint_start = group.get_current_joint_values()
    print("Starting joint values: ", joint_start)
    joint_goal = [pi/4, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, pi/4, 0.0, 0.0, 0.0]
    print("Goal joint values: ", joint_goal)

    joint_names = ['joint_2', 'joint_3', 'joint_4', 'joint_5', \
                'joint_6', 'joint_7', 'joint_2_m', 'joint_3_m', \
                'joint_4_m', 'joint_5_m', 'joint_6_m', 'joint_7_m']
    
    group.set_joint_value_target(joint_goal)
    plan = group.plan()
    traj = plan[1]
    
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = joint_names
    joint_state.position = joint_start

    display_trajectory = DisplayTrajectory()
    display_trajectory.model_id = 'dual_arm'
    display_trajectory.trajectory.append(traj)
    display_trajectory.trajectory_start.joint_state = joint_state

    message = FollowJointTrajectoryActionGoal()
    message.goal.trajectory = traj
    # print(rospy.get_param('/abb_control/sim'))
    # if rospy.get_param('/abb_control/sim') == True:
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     message.goal.trajectory.header = header

    pub.publish(message)
    display_pub.publish(display_trajectory)
    execution = group.go(wait=True)
    print("Executed? ", execution)
    group.stop()
    group.clear_pose_targets()

if __name__ == '__main__':

    try:
        functional()
    except rospy.ROSInterruptException:
        pass