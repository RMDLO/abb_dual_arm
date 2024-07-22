#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>

using json = nlohmann::json;
using MoveItErrorCode = moveit::core::MoveItErrorCode;
namespace fs = boost::filesystem;

visualization_msgs::MarkerArray createMarkerArray(const std::vector<std::vector<double>>& points, double min_frac, const std::string& frame_id);
visualization_msgs::MarkerArray publishConfigPoints(const std::string& config_file);

int main(int argc, char** argv) {
    ros::init(argc, argv, "abb_compute_reachability");
    ros::NodeHandle nh;

    std::string path;
    bool config_file = false;  // Default value

    if (!nh.getParam("/abb_compute_reachability/load_reachability_config", config_file)) {
        ROS_ERROR("Failed to get parameter '/abb_compute_reachability/load_reachability_config'");
    }
    else {
        ROS_INFO("Loaded reachability config file.");
    }
    if (!nh.getParam("/abb_compute_reachability/abb_control_path", path)) {
        ROS_ERROR("Failed to get parameter '/abb_compute_reachability/abb_control_path'");
    }
    else {
        ROS_INFO("Loaded abb_control path: %s", path.c_str());
    }

    std::string config_path = path + "/config";
    std::string config_file_path = config_path + "/config.json";
    std::string reachable_file_path = config_path + "/reachable.json";
    ROS_INFO("Config file path: %s", config_file_path.c_str());
    ROS_INFO("Reachable file path: %s", reachable_file_path.c_str());

    visualization_msgs::MarkerArray marker_array;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    if (config_file) {
        ros::Publisher reachable_pub = nh.advertise<visualization_msgs::MarkerArray>("/abb_compute_reachability/reachability", 10);
        while (ros::ok()) {
            visualization_msgs::MarkerArray marker_array = publishConfigPoints(config_file_path);
            reachable_pub.publish(marker_array);
            ros::Duration(1.0).sleep();
        }
    } else {
        try {
            // Check if "config" directory already exists
            if (fs::exists(config_path)) {
                ROS_INFO("Config directory already exists at: %s", config_path.c_str());
            } else {
                // Create directory "config"
                if (fs::create_directory(config_path)) {
                    ROS_INFO("Config directory created successfully at: %s", config_path.c_str());
                } else {
                    ROS_ERROR("Failed to create config directory at: %s", config_path.c_str());
                }
            }
        } catch (const fs::filesystem_error& e) {
            ROS_ERROR_STREAM("Failed to create config directory: " << e.what());
        }

        moveit::planning_interface::MoveGroupInterface move_group("mp_m");
        move_group.setPlannerId("RRTConnectkConfigDefault");

        std::vector<double> bounds = {-0.3, 0.5, -0.4, 0.4, 0.0, 0.6};
        double resolution = 0.1;
        std::vector<std::vector<double>> grid_points;
        std::vector<std::vector<double>> reachable_points;
        double min_frac = 1.0;

        for (double x = bounds[0]; x < bounds[1]; x += resolution) {
            for (double y = bounds[2]; y < bounds[3]; y += resolution) {
                for (double z = bounds[4]; z < bounds[5]; z += resolution) {
                    std::vector<double> point = {x, y, z};

                    geometry_msgs::Pose pose_target;
                    pose_target.position.x = point[0];
                    pose_target.position.y = point[1];
                    pose_target.position.z = point[2];
                    pose_target.orientation.w = 1.0;

                    std::vector<geometry_msgs::Pose> waypoints;
                    waypoints.push_back(move_group.getCurrentPose().pose);
                    waypoints.push_back(pose_target);
                    moveit_msgs::RobotTrajectory trajectory;

                    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, true);

                    // Update min_frac if a smaller fraction is found
                    if (fraction < min_frac) {
                        min_frac = fraction;
                    }

                    std::vector<double> point_list = {x, y, z, fraction};

                    if (fraction == 1.0){
                        reachable_points.push_back(point_list);
                        ROS_INFO("Cartesian path computed successfully.");
                    } else {
                        ROS_WARN("Failed to compute Cartesian path");
                    }

                    grid_points.push_back(point_list);

                }
            }
        }

        // Save config data to JSON file
        std::ofstream config_json_file(config_file_path);
        if (!config_json_file.is_open()) {
            ROS_ERROR("Failed to open file: %s", config_file_path);
        }
        try {
            config_json_file << "{\n";
            config_json_file << "    \"fractions\": [\n";
            for (size_t i = 0; i < grid_points.size(); ++i) {
                auto& point = grid_points[i];
                config_json_file << "        [" << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << "]";
                if (i != grid_points.size() - 1) {
                    config_json_file << ",";
                }
                config_json_file << "\n";
            }
            config_json_file << "    ],\n";
            config_json_file << "    \"min_frac\": " << min_frac << "\n";
            config_json_file << "}\n";
        } catch (const std::exception& e) {
            ROS_ERROR("Error writing to JSON file: %s", e.what());
        }

        config_json_file.close();

        // Save reachability data to JSON file
        std::ofstream reachable_json_file(reachable_file_path);
        if (!reachable_json_file.is_open()) {
            ROS_ERROR("Failed to open file: %s", reachable_file_path);
        }
        try {
            reachable_json_file << "{\n";
            reachable_json_file << "    \"points\": [\n";
            for (size_t i = 0; i < reachable_points.size(); ++i) {
                auto& point = reachable_points[i];
                reachable_json_file << "        [" << point[0] << ", " << point[1] << ", " << point[2] << "]";
                if (i != reachable_points.size() - 1) {
                    reachable_json_file << ",";
                }
                reachable_json_file << "\n";
            }
            reachable_json_file << "    ],\n";
            reachable_json_file << "}\n";
        } catch (const std::exception& e) {
            ROS_ERROR("Error writing to JSON file: %s", e.what());
        }

        reachable_json_file.close();

        // Create marker array for visualization
        marker_array = createMarkerArray(grid_points, 1.0, "world");

        // Publish marker arrays to RViz
        ros::Publisher reachable_pub = nh.advertise<visualization_msgs::MarkerArray>("/abb_compute_reachability/reachability", 10);
        while (ros::ok()) {
            reachable_pub.publish(marker_array);
            ros::Duration(1.0).sleep();
        }
    }

    ros::shutdown();
    return 0;
}

visualization_msgs::MarkerArray createMarkerArray(const std::vector<std::vector<double>>& points, double min_frac, const std::string& frame_id) {
    visualization_msgs::MarkerArray marker_array;
    size_t marker_id = 0;

    for (const auto& point : points) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "reachable_points";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point[0];
        marker.pose.position.y = point[1];
        marker.pose.position.z = point[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 0.3;

        // Interpolate color based on planning fraction value
        double red = 1.0 - (point[3] - min_frac) / (1.0 - min_frac);
        double green = 0.0;
        double blue = (point[3] - min_frac) / (1.0 - min_frac);
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;

        marker_array.markers.push_back(marker);
        marker_id++;
    }

    return marker_array;
}

visualization_msgs::MarkerArray publishConfigPoints(const std::string& config_file) {
    visualization_msgs::MarkerArray marker_array;

    std::ifstream file(config_file);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open config file: %s", config_file.c_str());
        return marker_array;
    }

    // Parse JSON file
    nlohmann::json reachability;
    file >> reachability;
    file.close();

    if (!reachability.contains("fractions") || !reachability.contains("min_frac")) {
        ROS_ERROR("Invalid JSON format");
        return marker_array;
    }

    marker_array = createMarkerArray(reachability["fractions"], reachability["min_frac"], "world");

    return marker_array;
}