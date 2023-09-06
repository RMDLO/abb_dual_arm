#!/usr/bin/env python3

import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Pose, Quaternion
import copy

def functional():

    group_name = "mp_m"
    rospy.init_node(f'{group_name}_cartesian_planning')
    group= moveit_commander.MoveGroupCommander(group_name)
    group.set_start_state_to_current_state()

    pub = rospy.Publisher(f'/{group_name}/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=1)
    display_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)

    joint_start = group.get_current_joint_values()
    waypoints = []
    scale=1

    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

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
    joint_state.position = joint_start

    display_trajectory = DisplayTrajectory()
    display_trajectory.model_id = group_name
    display_trajectory.trajectory.append(plan)
    display_trajectory.trajectory_start.joint_state = joint_state

    message = FollowJointTrajectoryActionGoal()
    message.goal.trajectory = plan

    pub.publish(message)
    display_pub.publish(display_trajectory)
    execution = group.execute(plan, wait=True)
    print("Executed? ", execution)
    print("End Pose: ", group.get_current_pose().pose)
    group.stop()
    group.clear_pose_targets()

if __name__ == '__main__':

    try:
        functional()
    except rospy.ROSInterruptException:
        pass