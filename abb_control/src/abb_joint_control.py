#!/usr/bin/env python3

from numpy import pi
import argparse

import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal

def functional(config):

    if config == "mp_m_initial":
        name = "mp_m"
    else:
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
        elif config == 'initial1':
            joint_goal = [-5.828056146128802e-06, 0.954073429107666, -1.1964266300201416, 9.70017135841772e-05, 0.2419261336326599, 6.600239430554211e-05, 0, 0.0, 0, 0.0, pi/2, 0.0]
        elif config == 'initial2':
            joint_goal = [-9.298604709329084e-05, 0.08021783083677292, -0.1256103366613388, 0.00012576385051943362, -0.23865161836147308, 0.0003152742865495384, 0, 0.0, 0, 0.0, pi/2, 0.0]
        elif config == 'initial3':
            joint_goal = [-1.7713238776195794e-05, 0.791336715221405, -0.9546111226081848, 0.0006722444668412209, -0.009530151262879372, -0.0004804782220162451, 0, 0.0, 0, 0.0, pi/2, 0.0]
        elif config == 'initial4': # good for rope close to mp_m!!
            joint_goal = [8.766870450926945e-05, 0.6260115504264832, -0.8013136386871338, 3.372501305420883e-05, -0.11077476292848587, 7.175483187893406e-05, 0, 0.0, 0, 0.0, pi/2, 0.0]
        elif config == "closeup":
            joint_goal = [0.26027172803878784, 0.396650493144989, 0.040516164153814316, 0.31667420268058777, -0.8686138391494751, -0.3101566433906555, -7.051671127555892e-05, -2.6221681764582172e-05, -5.3057505283504725e-05, 5.773369048256427e-05, 1.5707331895828247, 1.704614442132879e-05]
        joint_names = ['joint_2', 'joint_3', 'joint_4', 'joint_5', \
            'joint_6', 'joint_7', 'joint_2_m', 'joint_3_m', \
            'joint_4_m', 'joint_5_m', 'joint_6_m', 'joint_7_m']
    elif name == "mp":
        joint_goal = [0.9268576502799988, 
                      0.2813063859939575, 
                      0.7393782734870911,
                      1.145723581314087, 
                      -1.0036907196044922,
                      -0.9744566082954407]
        joint_names = ['joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
    elif name == "mp_m":
        joint_goal = [0, 0.0, 0, 0.0, pi/2, 0.0]
        joint_names = ['joint_2_m', 'joint_3_m', 'joint_4_m', 'joint_5_m', 'joint_6_m', 'joint_7_m']

    print("Goal joint values: ", joint_goal)
    
    group.set_joint_value_target(joint_goal)
    _, plan, _, _ = group.plan()
    
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = joint_names
    joint_state.position = joint_start

    display_trajectory = DisplayTrajectory()
    display_trajectory.model_id = name
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
    parser.add_argument("config", help="Indicate configuration: ['all_zero', 'initial1', 'initial2', 'initial3', 'initial4', 'closeup', 'mp_m_initial']")

    args = parser.parse_args()

    try:
        functional(args.config)
    except rospy.ROSInterruptException:
        pass