/**
 * @file   abb_pose_control_node.cpp
 * @brief  This ROS node performs Cartesian path planning for an ABB IRB 120 industrial robot.
 *         The program uses MoveIt to control the robot and TF2 for coordinate transformations.
 *
 * Required ROS Libraries:
 * - ros/ros.h: ROS core library for C++.
 * - moveit/move_group_interface/move_group_interface.h: MoveIt interface for controlling the robot.
 * - tf2_ros/transform_listener.h: TF2 library to listen for coordinate transforms.
 * - tf2_geometry_msgs/tf2_geometry_msgs.h: TF2 library for geometry messages.
 * - sensor_msgs/JointState.h: ROS standard message type for reporting joint states.
 * - moveit_msgs/DisplayTrajectory.h: Message to display robot trajectory in Rviz.
 * - control_msgs/FollowJointTrajectoryActionGoal.h: Message to send joint trajectory goals.
 * - geometry_msgs/PoseStamped.h: Message to hold pose data with frame information.
 *
 * Workflow:
 * 1. Initialize ROS and create a node handle.
 * 2. Initialize MoveIt for a given planning group.
 * 3. Define ROS publishers for sending trajectory goals and visualizing the planned path.
 * 4. Set up TF2 to listen to transforms between frames.
 * 5. Create a vector to hold waypoints for the Cartesian path.
 * 6. Use TF2 to transform the target pose (marker frame) to the robot's planning frame.
 * 7. Compute the Cartesian path using MoveIt and get the fraction of the path that was successfully computed.
 * 8. If the path is viable (fraction >= 0.9), execute the plan. Otherwise, print a warning.
 *
 * Note: Replace the 'jump_threshold', 'eef_step' and other parameters according to your application needs.
 */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "modified_cartesian_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string group_name;
    std::cout << "Enter the group name (valid choices are: mp, mp_m, dual_arm): ";
    std::cin >> group_name;

    if (group_name != "mp" && group_name != "mp_m" && group_name != "dual_arm") {
        std::cout << "Invalid group name entered. Exiting..." << std::endl;
        return -1;
    }

    moveit::planning_interface::MoveGroupInterface group(group_name);
    group.setStartStateToCurrentState();

    robot_state::RobotStatePtr current_state = group.getCurrentState();
    ros::Duration(5.0).sleep();
    if (!current_state) {
        ROS_ERROR("Failed to fetch the current robot state.");
        return -1;
    }

    ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(1.0).sleep();

    std::string frame_name;
    std::cout << "Enter the name of the frame you want to transform to: ";
    std::cin >> frame_name;

    try {
        geometry_msgs::TransformStamped transform_stamped = tfBuffer.lookupTransform(group.getPlanningFrame(), frame_name, ros::Time(0), ros::Duration(1.0));
        geometry_msgs::PoseStamped marker_pose_stamped;
        marker_pose_stamped.header.frame_id = group.getPlanningFrame();

        marker_pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
        marker_pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
        marker_pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
        marker_pose_stamped.pose.orientation.x = transform_stamped.transform.rotation.x;
        marker_pose_stamped.pose.orientation.y = transform_stamped.transform.rotation.y;
        marker_pose_stamped.pose.orientation.z = transform_stamped.transform.rotation.z;
        marker_pose_stamped.pose.orientation.w = transform_stamped.transform.rotation.w;

        group.setPoseTarget(marker_pose_stamped.pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            moveit_msgs::DisplayTrajectory display_trajectory;
            display_trajectory.model_id = group_name;
            display_trajectory.trajectory.push_back(plan.trajectory_);
            display_pub.publish(display_trajectory);

            std::string input;
            std::cout << "Execute plan? y or n: ";
            std::cin >> input;

            if (input == "y") {
                group.move();
                std::cout << "Plan executed." << std::endl;
            } else {
                std::cout << "Trajectory execution on robot aborted." << std::endl;
            }
        } else {
            ROS_WARN("Could not compute a valid path.");
        }
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could not transform to target pose: %s", ex.what());
    }

    ros::shutdown();
    return 0;
}