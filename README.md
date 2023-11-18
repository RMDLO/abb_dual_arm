# ABB Robot Operating System (ROS) Package for Dual-Arm Motion Planning and Execution

<p>
  <a href="https://github.com/RMDLO/abb_dual_arm/actions/workflows/build.yml?query=branch%3Amaster" alt="GitHub Actions">
    <img src="https://img.shields.io/github/actions/workflow/status/RMDLO/abb_dual_arm/build.yml?branch=master">
  </a>
</p>

This repository contains ROS drivers to perform dual-arm coordinated motion planning for two ABB IRB120 robots in both hardware and simulation. Before getting started, please read the [ABB IRB120 product manual](https://library.e.abb.com/public/35c8d30aebad4d13b945a1943e354ac5/3HAC035728%20PM%20IRB%20120-en.pdf) which contains important safety information.

Current Development Status:

- Dual-arm coordinated (synchronous) motion planning and execution with MoveIt - works as expected.
- Hardware interface - works as expected.
- Collision checking - works as expected.

https://user-images.githubusercontent.com/93821405/205412434-5ec18dfc-8c36-49a5-9855-a32c7595761e.mp4

## Install the ABB Dual Arm ROS Package

First, update the local rosdep database:

```bash
rosdep update
```

Clone the ABB robot ROS package into the `src` folder of a catkin workspace.

```bash
git clone git@github.com:RMDLO/abb_dual_arm.git --recurse-submodules
```

## Install ROS Dependencies and Build the Package

Change directories to the root of the ABB catkin workspace and use rosdep to install any missing ROS dependencies.

```bash
cd .. && sudo rosdep install --from-paths src --ignore-packages-from-source --rosdistro noetic
```
Use catkin_tools to build the workspace:

```bash
catkin build
```

## Configure the Robot IRC5 controller for ROS Control

1. Turn on the robot IRC5 controller by turning the top left power switch to on. Wait until the teach pendant is on, and then switch the IRC5 controller to [automatic mode from manual mode](!http://wiki.ros.org/abb_driver/Tutorials/RunServer) by turning the key on the cabinet counterclockwise until the white status light turns off. On the robot teach pendant, verify it is okay to switch to automatic mode when prompted. Press the status light on the IRC5 control cabinet again until it turns on.
2.  On the teach pendant, select settings and verify the mode is continuous. Then select PP to Main. Then click the play button on the teach pendant. The play button signals the teach pendant to run the loaded RAPID module.
3. On the desktop computer, in Wired Settings, ensure the ethernet connection to the IRC5 controller is set to "Automatic (DHCP)" in the IPv4 setting.

## Use Docker (Optional)

To set up a docker container for running the robots in hardware and simulation, see the [DOCKER.md](https://github.com/RMDLO/abb_dual_arm/blob/master/docker/DOCKER.md) file.

## Move the Robots with RViz

After setting up the hardware for ROS control, open a new terminal. First, navigate into the root of the workspace and source the workspace to access the built packages.

```bash
source devel/setup.bash
```
Launch MoveIt! planning and execution. Make sure to change the robot controller's IP address in `moveit_planning_execution.launch` and set `sim:=False` if controlling the real robots.
```bash
roslaunch abb_irb120_moveit_config moveit_planning_execution.launch sim:=True
```
The robots can be controlled through click-and-dragging within the RViz interface.

## Move a Robot to Specified Joint Angles

After setting up the robot controller for ROS control, open two new terminals and perform the below commands in the root of the workspace to move the robots to specified joint angles.

#### Terminal 1:

Source the workspace to access the built packages.
```bash
source devel/setup.bash
```
Launch MoveIt! planning and execution. Make sure to change the robot controller's IP address in `abb_control.launch` and set `sim:=False` if controlling the real robots. If using the pointer as the end effector, set `pointer:=True` on launch.
```bash
roslaunch abb_control abb_control.launch sim:=True camera:=False pointer:=False
```

#### Terminal 2

Change desired joint angle values in `abb_control.py` first. Then source the workspace.
```bash
source devel/setup.bash
```
Run the robot control node.
```bash
rosrun abb_control abb_joint_control.py
```

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
