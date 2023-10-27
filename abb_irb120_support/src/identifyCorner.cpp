#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

int main() {
    // Load the image
    std::string image_path = "/home/raghav/cws/abb_dual_arm/src/abb_dual_arm/abb_irb120_support/20231027/000_image.jpg";
    cv::Mat image = cv::imread(image_path);

    // Check if the image is loaded
    if (image.empty()) {
        std::cout << "Image not loaded. Check if the file exists at " << image_path << std::endl;
        return -1;
    }

    // Convert the image to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Optionally, apply Gaussian blur to reduce noise
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

    // Define Charuco dictionary and board
    cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    float square_length = 0.024f;  // Adjust based on board, in meters
    float marker_length = 0.018f;  // Adjust based on board, in meters
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board = cv::aruco::CharucoBoard::create(5, 7, square_length, marker_length, aruco_dict);

    // Detect Aruco markers
    std::vector<std::vector<cv::Point2f>> markers;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejectedImgPoints;
    cv::aruco::detectMarkers(blurred, aruco_dict, markers, ids, cv::aruco::DetectorParameters::create(), rejectedImgPoints);

    // Interpolate Charuco corners
    cv::Mat charuco_corners, charuco_ids;
    cv::aruco::interpolateCornersCharuco(markers, ids, blurred, charuco_board, charuco_corners, charuco_ids);

    // Print all detected Charuco corner IDs
    if (charuco_ids.empty()) {
        std::cout << "No Charuco corners were detected." << std::endl;
    } else {
        std::cout << "Detected Charuco corner IDs: ";
        for (int i = 0; i < charuco_ids.rows; ++i) {
            std::cout << charuco_ids.at<int>(i, 0);
            if (i < charuco_ids.rows - 1) {
                std::cout << ", ";
            }
        }
        std::cout << std::endl;

        // Ask the user for the desired corner ID
        int desired_id;
        std::cout << "Enter the desired corner ID: ";
        std::cin >> desired_id;

        // Find the index of the desired corner ID
        int index = -1;
        for (int i = 0; i < charuco_ids.rows; ++i) {
            if (charuco_ids.at<int>(i, 0) == desired_id) {
                index = i;
                break;
            }
        }

        if (index != -1) {
            std::cout << "Desired corner ID " << desired_id << " found at index " << index << std::endl;
        } else {
            std::cout << "Desired corner ID " << desired_id << " not found." << std::endl;
        }

        if (index >= 0) {
            cv::Point2f corner_coordinates = charuco_corners.at<cv::Point2f>(index, 0);
            std::cout << "Corner coordinates for ID " << desired_id << ": " << corner_coordinates << std::endl;

            // Draw a circle at the corner coordinates
            cv::circle(image, corner_coordinates, 4, cv::Scalar(0, 0, 255), -1);

            // Save display the modified image
            cv::imwrite("/home/raghav/cws/abb_dual_arm/src/abb_dual_arm/abb_irb120_support/20231027/000_image_with_marker.jpg", image);

        } else {
            std::cout << "Desired corner ID not found" << std::endl;
        }

        // Directly use the provided camera matrix and distortion coefficients
        cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 612.2394409179688, 0.0, 323.92718505859375, 0.0, 610.8439331054688, 236.01596069335938, 0.0, 0.0, 1.0);
        cv::Mat dist_coeffs = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);

        // Print the matrices to verify the values
        std::cout << "Camera Matrix:" << std::endl << camera_matrix << std::endl;
        std::cout << "Distortion Coefficients:" << std::endl << dist_coeffs << std::endl;

        // Estimate pose of the board
        cv::Vec3d rvec, tvec;
        bool validPose = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, charuco_board, camera_matrix, dist_coeffs, rvec, tvec);

        if (validPose) {
            std::cout << "Rotation vector (rvec): " << rvec << std::endl;
            std::cout << "Translation vector (tvec): " << tvec << std::endl;

            // Transform the 3D point from the board's coordinate system to the camera's coordinate system
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);  // Convert rotation vector to rotation matrix

            // To get rvec and tvec of the desired corner ID
            cv::Point3f desired_corner_3d = charuco_board->chessboardCorners[desired_id];
            cv::Mat desired_corner_mat = (cv::Mat_<double>(3, 1) << desired_corner_3d.x, desired_corner_3d.y, desired_corner_3d.z);
            cv::Mat tvec_mat = cv::Mat(tvec);
            cv::Mat transformed_corner = rotation_matrix * desired_corner_mat + tvec_mat;
            
            // Output transformed corner
            std::cout << "3D coordinates of the desired corner (ID " << desired_id << ") in camera's coordinate system: ";
            std::cout << transformed_corner.t() << std::endl;
            
            // Output rvec and tvec of the desired corner (with respect to the camera frame)
            std::cout << "Rotation vector (rvec) for the desired corner (ID " << desired_id << "): " << rvec << std::endl; // Stays the same as the board's rvec
            std::cout << "Translation vector (tvec) for the desired corner (ID " << desired_id << "): " << transformed_corner.t() << std::endl;

            // Write rvec and tvec to a text file
            std::ofstream output_file("charuco_corner_rvec_tvec.txt");
            if (output_file.is_open()) {
                output_file << "rvec [";
                for (int i = 0; i < 3; ++i) {
                    output_file << rvec[i];
                    if (i < 2) output_file << ", ";
                }
                output_file << "]\n";

                // Here, we use transformed_corner.t() as the tvec for the desired corner.
                cv::Mat transformed_corner_transposed = transformed_corner.t();
                output_file << "tvec [";
                for (int i = 0; i < transformed_corner_transposed.cols; ++i) {
                    output_file << transformed_corner_transposed.at<double>(0, i);  
                    if (i < transformed_corner_transposed.cols - 1) output_file << ", ";
                }
                output_file << "]\n";

                output_file.close();
            } else {
                std::cout << "Could not open output file.\n";
            }
        } else {
            std::cout << "Invalid pose. Could not estimate the pose." << std::endl;
        }
    }

    return 0;
}