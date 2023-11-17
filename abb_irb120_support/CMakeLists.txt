cmake_minimum_required(VERSION 2.8.3)
project(abb_irb120_support)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp moveit_core moveit_ros_planning_interface tf2_ros)

# Declare a catkin package
catkin_package()

# Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare C++ executable, along with its source files
add_executable(abb_pose_control_node src/abb_pose_control_node.cpp)
add_executable(camera_optical_frames src/camera_optical_frames.cpp)

# To compile the node, you must also link it against libraries
target_link_libraries(abb_pose_control_node ${catkin_LIBRARIES})
target_link_libraries(camera_optical_frames ${catkin_LIBRARIES})

# Additional install commands
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})