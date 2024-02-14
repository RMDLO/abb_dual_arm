#!/usr/bin/env python3
import time
from pyquaternion import Quaternion
import math
import yaml
import os
from datetime import datetime
from pathlib import Path
import sys

import rospy
import logging
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
    def __init__(self, group_name, args):
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
        self.move = args

    def callback(self):

        self.group.set_start_state_to_current_state()

        # t0, q0 = self.get_tf('idx_0') # blue tip
        t50, q50 = self.get_tf('idx_50')
        # t100, q100 = self.get_tf('idx_100') # green tip
        tmid1, qmid1 = self.get_tf('idx_mid1')
        print(self.move)

        if self.move == "r1":
            self.r1_twist(t50, q50, tmid1)
        elif self.move == "r2": 
            # self-occluded - comment these if not performing r1!
            tmid2, qmid2 = self.get_tf('idx_mid2')
            tmid3, qmid3 = self.get_tf('idx_mid3') # occluded node
            tmid4, qmid4 = self.get_tf('idx_mid4') # tip closest to occluded node
            self.r2_slide(tmid2, qmid2, tmid3, tmid4, qmid4)
        elif self.move == "x": 
            tmid4, qmid4 = self.get_tf('idx_mid4') # tip closest to occluded node
            tmid5, qmid5 = self.get_tf('idx_mid5')
            self.pick_place_green_tip(tmid4,qmid4,tmid5,qmid5)

    def get_tf(self, from_frame):
        target_pt_tf = self.buffer.lookup_transform('world', from_frame, rospy.Time(0), timeout=rospy.Duration(1))
        t  = numpify(target_pt_tf.transform.translation)
        q = numpify(target_pt_tf.transform.rotation)
        return t, q

    def r1_twist(self, t, q, tmid):
        self.gripper_client("o")
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

        # Pose 2: grasp
        pose_goal.position.z = 0.04
        self.execute(pose_goal)
        self.gripper_client("c")

        # Pose 3: pre-twist
        pose_goal.position.x = tmid[0]
        pose_goal.position.y = tmid[1]
        pose_goal.position.z = tmid[2]
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

        # Pose 5: place at midpoint between pick and twist points
        pose_goal.position.x = 0.5*(t[0] + tmid[0])
        pose_goal.position.y = 0.5*(t[1] + tmid[1])
        pose_goal.position.z = 0.15
        success = self.execute(pose_goal)
        self.gripper_client("o")

        # Pose 6: default 
        self.default()

        return success

    def r2_slide(self, t, q, t_occluded, t_tip, q_tip):
        self.gripper_client("o")
        # Pose 1: pre-grasp
        pose_goal = Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        pose_goal.position.x = t[0]
        pose_goal.position.y = t[1]
        pose_goal.position.z = t[2] + 0.05
        success1 = self.execute(pose_goal)
        logging.info(f"Pre-grasp successfully executed? {success1}")
        
        # Pose 2: grasp
        pose_goal.position.z = 0.043
        success2 = self.execute(pose_goal)
        logging.info(f"Grasp successfully executed? {success2}")
        self.gripper_client("c")
        time.sleep(0.1)

        # Pose 3: pick up
        pose_goal.position.z += 0.13
        success3 = self.execute(pose_goal)
        logging.info(f"Pick successfully executed? {success3}")

        # Pose 4: slide
        # pose_goal.position.x = t_occluded[0]
        # pose_goal.position.y = t_occluded[1]
        # self.execute(pose_goal)

        # rotation_quaternion = Quaternion(axis=[0, 0, 1], angle=-math.pi/2)
        # pose_quat = Quaternion(w=q100[3], x=q100[0], y=q100[1], z=q100[2])
        # rotated = rotation_quaternion*pose_quat
        # print("rotation_quaternion", rotation_quaternion)
        # print("pose_quat: ", pose_quat)
        # print("rotated: ", rotated)
        # pose_goal.orientation.x = rotated.x
        # pose_goal.orientation.y = rotated.y
        # pose_goal.orientation.z = rotated.z
        # pose_goal.orientation.w = rotated.w

        # pose_goal.orientation.x = q100[0]
        # pose_goal.orientation.y = q100[1]
        # pose_goal.orientation.z = q100[2]
        # pose_goal.orientation.w = q100[3]
        # self.execute(pose_goal)

        # Pose 5: pre-place
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        pose_goal.position.x = t_occluded[0]
        pose_goal.position.y = t_occluded[1]
        pose_goal.position.z = t_occluded[2]
        success4 = self.execute(pose_goal)
        logging.info(f"Pre-place successfully executed? {success4}")

        # Pose 6: place
        pose_goal.position.x = t_tip[0]
        pose_goal.position.y = t_tip[1]
        pose_goal.position.z = 0.1 # this length depends on the length of the loop!
        success5 = self.execute(pose_goal)
        logging.info(f"Place successfully executed? {success5}")
        self.gripper_client("o")

        # Pose 6: default 
        self.default()

    def pick_place_green_tip(self, t, q, t_target, q_target):
        self.gripper_client("100") # close gripper to 10mm width 
        time.sleep(0.1)
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
        time.sleep(0.1)

        # Pose 2: grasp
        pose_goal.position.z = 0.043
        self.execute(pose_goal)
        self.gripper_client("o")
        self.gripper_client("c")
        time.sleep(0.1)

        # Pose 3: pick
        pose_goal.position.z += 0.2
        self.execute(pose_goal)

        # Pose 5: place
        pose_goal.position.x = t_target[0]
        pose_goal.position.y = t_target[1]
        pose_goal.position.z -= 0.1
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
        return success
    
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

    parser = rospy.myargv(argv=sys.argv) #r1, r2, or x
    arg = parser[1]

    c = CalibrationChecker(group_name, arg)
    # while not rospy.is_shutdown():
    try:
        c.callback()
    except KeyboardInterrupt:
        print("Shutting down")