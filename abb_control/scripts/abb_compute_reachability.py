#!/usr/bin/env python3

import rospy
import moveit_commander
import numpy as np
import json
import os
import sys
import cv2
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import tf2_ros
import tf
from ros_numpy import numpify

def create_reachability_markers(points, min_frac=0.0, frame_id="world"):
    marker_array = MarkerArray()
    marker_id = 0
    for point in points:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "reachable_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 0.3
        # Interpolate color based on planning fraction value
        red = 1 - (point[3] - min_frac) / (1.0 - min_frac)
        green = 0
        blue = (point[3] - min_frac) / (1.0 - min_frac)
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker_array.markers.append(marker)
        marker_id += 1
    return marker_array

def create_reachability_mask(points, img_dim, min_frac=0.0):
    T = cam_to_world_tf_lookup()
    dot_size = 20 # pixels
    dot = np.zeros((dot_size, dot_size, 3), dtype=np.uint8)
    reachability_image = np.zeros((img_dim[0], img_dim[1], 3), dtype=np.uint8)
    points_3d = np.array(points)
    points_3d_hom = np.hstack((points_3d[:,:-1], np.ones((points_3d.shape[0], 1))))
    points_2d = np.dot(PROJECTION_MATRIX, np.dot(T, points_3d_hom.T)).T
    points_2d = (points_2d[:, :2] / points_2d[:, 2:]).astype(int)  # Normalize by z-coordinate
    for i in range(len(points)):
        # Interpolate color based on planning fraction value
        red = 1 - (points[i][3] - min_frac) / (1.0 - min_frac)
        green = 0
        blue = (points[i][3] - min_frac) / (1.0 - min_frac)
        dot[:] = [red*255, green*255, blue*255]
        start_u = points_2d[i,0] - dot_size // 2
        start_v = points_2d[i,1] - dot_size // 2
        end_u = start_u + dot_size
        end_v = start_v + dot_size
        if (start_u > 0 and end_u < img_dim[0]) and (start_v > 0 and end_v < img_dim[1]):
            reachability_image[start_u:end_u, start_v:end_v] = dot

    return reachability_image

def pub_reachability_image(imgMsg):
    bridge = CvBridge()
    current_image = bridge.imgmsg_to_cv2(imgMsg)
    combined_image = cv2.addWeighted(current_image, 0.7, REACHABILITY_IMAGE, 0.3, 0)
    REACHABILITY_IMG_MSG = bridge.cv2_to_imgmsg(combined_image, encoding="rgb8")
    
    reachable_image_pub = rospy.Publisher('/abb_control/reachability_img', Image, queue_size=10)
    reachable_image_pub.publish(REACHABILITY_IMG_MSG)

def camera_info_callback(info: CameraInfo) -> None:
    """
    Converts CameraInfo message to an array and sets it as a global variable.
    Args:
        info (sensor_msgs.msg.CameraInfo): An RGB CameraInfo message
    """
    global PROJECTION_MATRIX
    PROJECTION_MATRIX = np.array(list(info.P)).reshape(3, 4)
    camera_info_sub.unregister()

def cam_to_world_tf_lookup():
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    msg = buffer.lookup_transform('world',
                                    'camera_color_optical_frame',
                                    rospy.Time(0),
                                    timeout=rospy.Duration(1))
    translation = numpify(msg.transform.translation)
    translation_matrix = tf.transformations.translation_matrix(translation)
    rotation_matrix = tf.transformations.quaternion_matrix(numpify(msg.transform.rotation))
    T = translation_matrix.copy()
    T[0:3,0:3] = rotation_matrix[0:3,0:3]
    return T

if __name__ == '__main__':

    camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_callback)

    path = rospy.get_param('/reachability/abb_dual_arm_path')
    config = rospy.get_param('/reachability/load_reachability_config')

    rospy.init_node('reachability', anonymous=True)

    file_path = f"{path}" + "/config/config.json"

    if config:
        bridge = CvBridge()
        try:
            with open(file_path, 'r') as file:
                reachability = json.load(file)
        except json.JSONDecodeError:
            print("Error parsing JSON data.")

        marker_array = create_reachability_markers(reachability["fractions"][:-1], min_frac=reachability["min_frac"])
        global REACHABILITY_IMAGE
        REACHABILITY_IMAGE = create_reachability_mask(reachability["fractions"][:-1], [480, 640], min_frac=reachability["min_frac"])

        reachable_marker_pub = rospy.Publisher('/abb_control/reachability', MarkerArray, queue_size=10)
        reachable_mask_pub = rospy.Publisher('/abb_control/reachability_mask', Image, queue_size=10)

        rospy.Subscriber('/camera/color/image_raw', Image, pub_reachability_image)    
      
        image_mask_msg = bridge.cv2_to_imgmsg(REACHABILITY_IMAGE, encoding="rgb8")
        while not rospy.is_shutdown():
            reachable_marker_pub.publish(marker_array)
            reachable_mask_pub.publish(image_mask_msg)
                
    else:
        try: 
            os.mkdir(f"{path}/config/")
        except FileExistsError:
            print("config directory already exists!")

        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)

        group = moveit_commander.MoveGroupCommander("mp_m")

        # Define the workspace boundaries and resolution in world frame
        bounds = [-0.3, 0.5, -0.3, 0.4, 0.0, 0.6]
        resolution = 0.1
        x_vals = np.arange(bounds[0], bounds[1], resolution)
        y_vals = np.arange(bounds[2], bounds[3], resolution)
        z_vals = np.arange(bounds[4], bounds[5], resolution)
        grid_points = np.array(np.meshgrid(x_vals, y_vals, z_vals)).T.reshape(-1, 3)

        reachability = {"fractions": [],
                        "min_frac": 1.0}

        for point in grid_points:
            pose_target = Pose()
            pose_target.position.x = point[0]
            pose_target.position.y = point[1]
            pose_target.position.z = point[2]
            pose_target.orientation.w = 1.0

            waypoints = []
            waypoints.append(group.get_current_pose().pose)
            waypoints.append(pose_target)
            (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0, avoid_collisions=True)
            if fraction!=1.0 and fraction < reachability["min_frac"]:
                reachability["min_frac"] = fraction
            point_list = point.tolist()
            point_list.append(fraction)
            reachability["fractions"].append(point_list)
        
        with open(file_path, 'w') as json_file:
            json.dump(reachability, json_file, indent=4)

        # Create marker array for visualization
        marker_array = create_reachability_markers(reachability["fractions"][:-1], min_frac=reachability["min_frac"])
        
        # Publish marker arrays to RViz
        reachable_pub = rospy.Publisher('/abb_control/reachability', MarkerArray, queue_size=10)
        while not rospy.is_shutdown():
            reachable_pub.publish(marker_array)
            rospy.sleep(1)

        moveit_commander.roscpp_shutdown()
