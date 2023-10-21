#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "abb_irb120_cartesian_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string group_name = "mp_m";
    moveit::planning_interface::MoveGroupInterface group(group_name);
    group.setStartStateToCurrentState();

    ros::Publisher pub = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/mp_m/joint_trajectory_action/goal", 1);
    ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration(1.0).sleep(); 

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(group.getCurrentPose().pose);

    try
    {
        // This will transform the marker's pose to the planning frame
        geometry_msgs::TransformStamped transform_stamped = tfBuffer.lookupTransform(group.getPlanningFrame(), "marker_frame", ros::Time(0), ros::Duration(1.0));

        geometry_msgs::PoseStamped marker_pose_stamped;
        marker_pose_stamped.header.frame_id = group.getPlanningFrame();
        
        // Populate the pose information from the TransformStamped object
        marker_pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
        marker_pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
        marker_pose_stamped.pose.position.z = transform_stamped.transform.translation.z;

        marker_pose_stamped.pose.orientation.x = transform_stamped.transform.rotation.x;
        marker_pose_stamped.pose.orientation.y = transform_stamped.transform.rotation.y;
        marker_pose_stamped.pose.orientation.z = transform_stamped.transform.rotation.z;
        marker_pose_stamped.pose.orientation.w = transform_stamped.transform.rotation.w;

        geometry_msgs::PoseStamped transformed_pose;
        tf2::doTransform(marker_pose_stamped, transformed_pose, transform_stamped);

        waypoints.push_back(transformed_pose.pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
 
        double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_, true);
    
        // Add this snippet here to check the fraction returned
        if (fraction >= 0.9)  
        {
            // Populate action goal and other steps before execution
            control_msgs::FollowJointTrajectoryActionGoal action_goal;
            action_goal.goal.trajectory = plan.trajectory_.joint_trajectory;

            pub.publish(action_goal);

            moveit_msgs::DisplayTrajectory display_trajectory;
            display_trajectory.model_id = group_name;
            display_trajectory.trajectory.push_back(plan.trajectory_);
            display_pub.publish(display_trajectory);
            
            std::string input;
            std::cout << "Execute plan? y or n: ";
            std::cin >> input;

            if (input == "y")
            {
                // Using 'group.move()' to execute previously computed plan
                moveit::core::MoveItErrorCode success_code = group.move();
                bool success = (success_code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                group.stop();
                group.clearPoseTargets();
                std::cout << "Executed? " << (success ? "Yes" : "No") << std::endl;
            }
            else
            {
            std::cout << "Trajectory execution on robot aborted." << std::endl;
            }
        }
        else
        {
            ROS_WARN("Could not compute a valid Cartesian path. Fraction returned: %f", fraction);
        }
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could not transform marker_pose: %s", ex.what());
    }

    ros::shutdown();

    return 0;
}