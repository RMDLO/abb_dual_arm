#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath> 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_frame_rotator");
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster br;

    while(ros::ok()) {
        geometry_msgs::TransformStamped color_transformStamped;
        geometry_msgs::TransformStamped depth_transformStamped;

        // For color frame
        color_transformStamped.header.stamp = ros::Time::now();
        color_transformStamped.header.frame_id = "camera_color_frame";
        color_transformStamped.child_frame_id = "camera_color_optical_frame";

        // No translation needed
        color_transformStamped.transform.translation.x = 0;
        color_transformStamped.transform.translation.y = 0;
        color_transformStamped.transform.translation.z = 0;

        tf2::Quaternion color_quat;
        color_quat.setRPY(0, 0, 0);  // 90-degree rotation about Z-axis
        color_transformStamped.transform.rotation.x = color_quat.x();
        color_transformStamped.transform.rotation.y = color_quat.y();
        color_transformStamped.transform.rotation.z = color_quat.z();
        color_transformStamped.transform.rotation.w = color_quat.w();

        // For depth frame
        depth_transformStamped.header.stamp = ros::Time::now();
        depth_transformStamped.header.frame_id = "camera_depth_frame";
        depth_transformStamped.child_frame_id = "camera_depth_optical_frame";

        // No translation needed
        depth_transformStamped.transform.translation.x = 0;
        depth_transformStamped.transform.translation.y = 0;
        depth_transformStamped.transform.translation.z = 0;

        tf2::Quaternion depth_quat;
        depth_quat.setRPY(0, 0, 0);  // 90-degree rotation about Z-axis
        depth_transformStamped.transform.rotation.x = depth_quat.x();
        depth_transformStamped.transform.rotation.y = depth_quat.y();
        depth_transformStamped.transform.rotation.z = depth_quat.z();
        depth_transformStamped.transform.rotation.w = depth_quat.w();

        // Send the transforms
        br.sendTransform(color_transformStamped);
        br.sendTransform(depth_transformStamped);

        ros::spinOnce();
        ros::Duration(0.1).sleep();  // To throttle the publishing rate
    }
    return 0;
}