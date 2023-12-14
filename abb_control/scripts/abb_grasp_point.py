#!/usr/bin/env python3
import time
from pyquaternion import Quaternion
import math
import numpy as np
import yaml
import os
from datetime import datetime
from pathlib import Path

import rospy
from ros_numpy import numpify
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Pose
import tf2_ros
from onrobot_2fg7_control.srv import SetCommand


class CalibrationChecker:
    '''
    Constructor for 'calibration_checker' node, a service call for trajectory planning (sawyer_planning) to a point on a 
    chessboard camera extrinsic calibration target. Depends on the 'eye_in_hand_broadcaster' node.
    '''
    def __init__(self, group_name):
        self.group_name = group_name
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.pub = rospy.Publisher(f'/{self.group_name}/joint_trajectory_action/goal', FollowJointTrajectoryActionGoal, queue_size=1)
        self.display_pub = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.gripper_client = rospy.ServiceProxy('/onrobot_2fg7/set_command', SetCommand)
        self.date = datetime.now().strftime("%Y%m%d")
        self.cal_data_path = Path(f'{Path(__file__).parent}/trajectories/{self.date}')
        if not self.cal_data_path.exists():
            self.cal_data_path.mkdir(parents=True, exist_ok=True)

    def callback(self):

        self.group.set_start_state_to_current_state()

        t0, q0 = self.get_tf('idx_0')
        t25, q25 = self.get_tf('idx_25')
        t50, q50 = self.get_tf('idx_50')
        t75, q75 = self.get_tf('idx_75')
        t100, q100 = self.get_tf('idx_100')
        tmid, qmid = self.get_tf('idx_mid')
        tmid2, qmid2 = self.get_tf('idx_mid2')

        self.r1_twist(t50, q50, tmid)
        self.r2_slide(t50, q50, tmid2)
        # self.x_cross(t25, q25)

    def get_tf(self, from_frame):
        target_pt_tf = self.buffer.lookup_transform('world', from_frame, rospy.Time(0), timeout=rospy.Duration(1))
        t  = numpify(target_pt_tf.transform.translation)
        q = numpify(target_pt_tf.transform.rotation)
        return t, q

    def r1_twist(self, t, q, tmid):
        # Pose 1: pre-grasp
        pose_goal = Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        pose_goal.position.x = t[0]
        pose_goal.position.y = t[1]
        pose_goal.position.z = t[2] + 0.1
        self.execute(pose_goal)
        time.sleep(0.2)

        # Pose 2: grasp
        pose_goal.position.z = 0.043
        self.execute(pose_goal)
        self.gripper_client("c")
        time.sleep(0.2)

        # Pose 3: pre-twist
        pose_goal.position.x = tmid[0]
        pose_goal.position.y = tmid[1]
        pose_goal.position.z += 0.2
        self.execute(pose_goal)

        # Pose 4: twist (for twisting RI)
        rotation_quaternion = Quaternion(axis=[0, 0, 1], angle=-math.pi)
        pose_quat = Quaternion(w=pose_goal.orientation.w, x=pose_goal.orientation.x, y=pose_goal.orientation.y, z=pose_goal.orientation.z)
        rotated = rotation_quaternion*pose_quat
        print("rotation_quaternion", rotation_quaternion)
        print("pose_quat: ", pose_quat)
        print("rotated: ", rotated)
        pose_goal.orientation.x = rotated.x
        pose_goal.orientation.y = rotated.y
        pose_goal.orientation.z = rotated.z
        pose_goal.orientation.w = rotated.w
        self.execute(pose_goal)

        # Pose 5: place
        pose_goal.position.x = t[0]
        pose_goal.position.y = t[1]
        pose_goal.position.z -= 0.2
        self.execute(pose_goal)
        self.gripper_client("o")

        # Pose 6: default 
        self.default()

    def r2_slide(self, t, q, tmid):
        # Pose 1: pre-grasp
        pose_goal = Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        pose_goal.position.x = t[0]
        pose_goal.position.y = t[1]
        pose_goal.position.z = t[2] + 0.1
        self.execute(pose_goal)
        time.sleep(0.2)

        # Pose 2: grasp
        pose_goal.position.z = 0.043
        self.execute(pose_goal)
        self.gripper_client("c")
        time.sleep(0.2)

        # Pose 3: slide
        pose_goal.position.x = tmid[0]
        pose_goal.position.y = tmid[1]
        pose_goal.position.z += 0.15
        self.execute(pose_goal)

        # Pose 5: place
        pose_goal.position.x = tmid[0]
        pose_goal.position.y = tmid[1]
        pose_goal.position.z -= 0.15
        self.execute(pose_goal)
        self.gripper_client("o")

        # Pose 6: default 
        self.default()

    def x_cross(self, t, q):
        # Pose 1: pre-grasp
        pose_goal = Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        pose_goal.position.x = t[0]
        pose_goal.position.y = t[1]
        pose_goal.position.z = t[2] + 0.1
        self.execute(pose_goal)
        time.sleep(0.2)

        # Pose 2: grasp
        pose_goal.position.z = 0.043
        self.execute(pose_goal)
        self.gripper_client("c")
        time.sleep(0.2)

        # Pose 3: pick
        pose_goal.position.z += 0.2
        self.execute(pose_goal)

        # Pose 5: place
        pose_goal.position.x = t[0]
        pose_goal.position.y = t[1]
        pose_goal.position.z -= 0.16
        self.execute(pose_goal)
        self.gripper_client("o")

        # Pose 6: default 
        self.default()

    def execute(self, pose_goal):

        self.group.set_pose_target(pose_goal)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        print("Executed? ", success)
    
    def default(self):
        joint_goal = [0, 0.0, 0, 0.0, math.pi/2, 0.0]
        self.group.set_joint_value_target(joint_goal)
        _, plan, _, _ = self.group.plan()
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        print("Executed? ", success)

    def save_plan(self, plan):
        file_path = os.path.join("" 'plan.yaml')
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

    def load_plan(self, path_to_plan):
        with open(path_to_plan, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        self.group.execute(loaded_plan)

if __name__ == '__main__':
    group_name = "mp_m"
    rospy.init_node('grasp_point')

    c = CalibrationChecker(group_name)
    # while not rospy.is_shutdown():
    try:
        c.callback()
    except KeyboardInterrupt:
        print("Shutting down")