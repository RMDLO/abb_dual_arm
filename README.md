# RMDLO ABB Robot Workspace for Dual-Arm Setup

This repository contains ROS drivers to perform dual-arm coordinated motion planning for two ABB IRB120 robots in both hardware and simulation. Before getting started, please read the [ABB IRB120 Product manual](https://library.e.abb.com/public/35c8d30aebad4d13b945a1943e354ac5/3HAC035728%20PM%20IRB%20120-en.pdf) which contains important safety information.

### For a comprehensive guide refer to the how_to.pdf which includes:

- Building a template repository for multi-arm robotics.
- Steps for adding models such as additional arms, sensors and cameras into ROS/MoveIt environment.
- Adding transforms - establishing common links between arms.
- Groups - an easier approach to duplicating existing systems with just a change in namespace.
- Topics - the need for remapping topics while using groups and namespace.

### Current status - 

- Dual-arm coordinated (synchronous) motion planning can be carried out using MoveIt.
- Hardware interface - works as expected.
- Collision checking works.  

### Coordinated Motion - Dual-Arm Setup

https://user-images.githubusercontent.com/93821405/205412434-5ec18dfc-8c36-49a5-9855-a32c7595761e.mp4

<!-- https://user-images.githubusercontent.com/93821405/204172896-e4cfaeb9-4eeb-4013-9db9-88399d4c3a16.mp4 -->

## Install the ABB Workspace for the first time
```bash
# First update the local rosdep database.
$ rosdep update
# Clone the ABB robot catkin workspace.
$ git clone git@github.com:RMDLO/abb_dual_arm.git --recurse-submodules
# Switch to noetic branch
$ cd abb_dual_arm && git checkout noetic
# Change to the root of the ABB catkin workspace.
$ cd ../..
# Use rosdep to install any missing dependencies.
$ sudo rosdep install --from-paths src --ignore-packages-from-source --rosdistro noetic
# Build the workspace (using catkin_tools).
$ catkin build
```

## Set up the hardware for ROS control

1. Turn on the robot IRC5 controller by turning the top left power switch to on. Wait until the teach pendant is on, and then switch the IRC5 controller to [automatic mode from manual mode](!http://wiki.ros.org/abb_driver/Tutorials/RunServer) by turning the key on the cabinet counterclockwise until the white status light turns off. On the robot teach pendant, verify it is okay to switch to automatic mode when prompted. Press the status light on the IRC5 control cabinet again until it turns on.
2.  On the teach pendant, select settings and verify the mode is continuous. Then select PP to Main. Then click the play button on the teach pendant. The play button signals the teach pendant to run the loaded RAPID module.
3. On the desktop computer, in Wired Settings, ensure the ethernet connection to the IRC5 controller is set to "Automatic (DHCP)" in the IPv4 setting. 

## Move a robot with the RViz interface

After setting up the hardware for ROS control, open a new terminal and perform:

```bash
# First activate the workspace to gain access to the built packages.
$ cd abb_ws && source devel/setup.bash
# Launch MoveIt! planning and execution using the robot's IP address.
$ roslaunch abb_irb120_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.125.1
```
<!-- ![Dual-Arm Setup (MoveIt)](images/a.png) -->

## Move a robot by specifying joint angles

After setting up the hardware for ROS control, open two new terminals and perform the below commands.

Terminal 1:

```bash
# First activate the workspace to gain access to the built packages.
$ cd abb_ws && source devel/setup.bash
# Launch MoveIt! planning and execution using the robot's IP address.
$ roslaunch abb_irb120_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.125.1
```
Terminal 2 (change desired joint angle values in `abb_control.py` first)
```bash
# First activate the workspace to gain access to the built packages.
$ cd abb_ws && source devel/setup.bash
# Launch MoveIt! planning and execution using the robot's IP address.
$ rosrun abb_irb120_moveit_config abb_control.py
```

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
