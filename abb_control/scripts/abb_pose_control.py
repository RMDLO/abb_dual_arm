#!/usr/bin/env python3

import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Pose, Quaternion

def functional():

    rospy.init_node('mp_m_cartesian_planning')
    group_name = "mp_m"
    group= moveit_commander.MoveGroupCommander(group_name)
    group.set_start_state_to_current_state()

    pub = rospy.Publisher('/dual_arm/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=1)
    display_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)

    joint_start = group.get_current_joint_values()
    start_pose = group.get_current_pose().pose
    print("Start pose: ", start_pose)

    goal_pose = Pose()
    goal_pose.position.x = 0.7
    goal_pose.position.y = 0
    goal_pose.position.z = 0.3
    goal_pose.orientation = Quaternion(0, 0, 0, 1)
    print("Goal pose: ", goal_pose)

    waypoints = []
    waypoints.append(start_pose)
    waypoints.append(goal_pose)

    (plan, _) = group.compute_cartesian_path(waypoints, 0.01, 0.0)  # 0.01 is the step size, 0.0 is the jump threshold

    if group_name == "dual_arm":
        joint_names = ['joint_2', 'joint_3', 'joint_4', 'joint_5', \
                    'joint_6', 'joint_7', 'joint_2_m', 'joint_3_m', \
                    'joint_4_m', 'joint_5_m', 'joint_6_m', 'joint_7_m']
    elif group_name == "mp_m":
        joint_names = ['joint_2_m', 'joint_3_m', 'joint_4_m', \
                       'joint_5_m', 'joint_6_m', 'joint_7_m']
    
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
    execution = group.go(wait=True)
    print("Executed? ", execution)
    group.stop()
    group.clear_pose_targets()

if __name__ == '__main__':

    try:
        functional()
    except rospy.ROSInterruptException:
        pass