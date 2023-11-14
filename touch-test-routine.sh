#!/bin/bash

# Source ROS environment
source ~/catkin_ws/devel/setup.bash

# Launch ROS nodes in separate Terminator windows
terminator -e "bash -c 'source ~/catkin_ws/devel/setup.bash; roslaunch abb_control abb_control.launch sim:=False pointer:=True; exec bash'" &
terminator -e "bash -c 'source ~/catkin_ws/devel/setup.bash; roslaunch realsense2_camera rs_camera.launch; exec bash'" &

# Wait for nodes to initialize
sleep 10

# Activate Python environment and run calibration in a new Terminator window
terminator -e "bash -c 'source ~/catkin_ws/devel/setup.bash; source numpyenvros/bin/activate; rosrun axbycz_data calibrate.py; deactivate; exec bash'" &

# Wait for calibration to complete
echo "Press any key once calibration is complete..."
read -n 1 -s

# Copy calibration data and compile/run identifyCorner in a new Terminator window
LATEST_FOLDER=$(ls -dt ~/src/axbycz_data/calibration_data/*/ | head -1)
cp -r "$LATEST_FOLDER" ~/src/abb_dual_arm/abb_irb120_support/
terminator -e "bash -c 'source ~/catkin_ws/devel/setup.bash; g++ ~/src/abb_dual_arm/abb_irb120_support/src/identifyCorner.cpp -o identifyCorner \`pkg-config --cflags --libs opencv4\`; ./identifyCorner; exec bash'" &

# Wait for corner identification to complete
echo "Press any key once corner identification is complete..."
read -n 1 -s

# Launch final move group in a new Terminator window
terminator -e "bash -c 'source ~/catkin_ws/devel/setup.bash; roslaunch abb_irb120_support abb_irb120_cartesian_planning.launch; exec bash'" &