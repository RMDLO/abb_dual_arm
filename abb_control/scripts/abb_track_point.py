#!/usr/bin/env python3

import rospy
from ros_numpy import numpify
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Pose
import tf2_ros

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

    def functional(self):

        self.group.set_start_state_to_current_state()

        waypoints = []
        waypoints.append(self.group.get_current_pose().pose)

        print("current pose: ", self.group.get_current_pose().pose)

        target_pt_tf = self.buffer.lookup_transform('world', 'target_point', rospy.Time(0), timeout=rospy.Duration(1))
        t  = numpify(target_pt_tf.transform.translation)
        q = numpify(target_pt_tf.transform.rotation)

        pose_goal1 = Pose()
        pose_goal1.orientation.x = q[0]
        pose_goal1.orientation.y = q[1]
        pose_goal1.orientation.z = q[2]
        pose_goal1.orientation.w = q[3]
        pose_goal1.position.x = t[0]
        pose_goal1.position.y = t[1]
        pose_goal1.position.z = 0.1

        pose_goal2 = Pose()
        pose_goal2.orientation.x = q[0]
        pose_goal2.orientation.y = q[1]
        pose_goal2.orientation.z = q[2]
        pose_goal2.orientation.w = q[3]
        pose_goal2.position.x = t[0]+0.05
        pose_goal2.position.y = t[1]
        pose_goal2.position.z = 0.1

        waypoints.append(pose_goal1)
        waypoints.append(pose_goal2)

        self.execute(pose_goal1)
        self.execute(pose_goal2)

    def execute(self, pose_goal):

        self.group.set_pose_target(pose_goal)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        print("Executed? ", success)

if __name__ == '__main__':
    group_name = "mp_m"
    rospy.init_node(f'{group_name}_cartesian_planning')

    c = CalibrationChecker(group_name)
    while not rospy.is_shutdown():
        try:
            c.functional()
        except KeyboardInterrupt:
            print("Shutting down")