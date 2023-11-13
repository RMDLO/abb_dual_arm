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
    ros::init(argc, argv, "simple_cartesian_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "mp_m";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.orientation.w = 1.0;
    target_pose.pose.position.x = 0.28;
    target_pose.pose.position.y = -0.2;
    target_pose.pose.position.z = 0.5;

    move_group_interface.setPoseTarget(target_pose.pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        std::cout << "Planning successful. Execute the plan? (y/n): ";
        char input;
        std::cin >> input;
        if (input == 'y' || input == 'Y')
        {
            move_group_interface.execute(my_plan);
        }
    }
    else
    {
        ROS_WARN("Planning failed.");
    }

    ros::shutdown();
    return 0;
}