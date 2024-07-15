cmake_minimum_required(VERSION 3.0.2)
project(abb_control)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    roscpp
    rospy
    std_msgs
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  # Include Boost headers
)

add_executable(reachability src/abb_compute_reachability.cpp)
target_link_libraries(reachability
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}  # Link Boost libraries
)

# Additional install commands
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

install(DIRECTORY config launch src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})