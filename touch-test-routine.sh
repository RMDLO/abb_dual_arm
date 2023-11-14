#!/bin/bash

# Source ROS environment
source ~/cws/abb_dual_arm/devel/setup.bash

# Launch ROS nodes in separate Terminator windows
terminator -e "bash -c 'source ~/cws/abb_dual_arm/devel/setup.bash; 
                roslaunch abb_control abb_control.launch sim:=False pointer:=True; 
                exec bash'" &

# Wait for nodes to initialize
sleep 25

terminator -e "bash -c 'source ~/cws/abb_dual_arm/devel/setup.bash; 
                roslaunch realsense2_camera rs_camera.launch; 
                exec bash'" &

# Wait for nodes to initialize
sleep 25

# Activate Python environment and run calibration in a new Terminator window
terminator -e "bash -c 'source ~/cws/abb_dual_arm/devel/setup.bash; 
                source numpyenvros/bin/activate; rosrun axbycz_data calibrate.py; 
                exec bash'" &

# Wait for data collection to complete
echo "Press any key once image capture is complete..."
read -n 1 -s

# Compile/run identifyCorner in a new Terminator window
terminator -e "bash -c 'source ~/cws/abb_dual_arm/devel/setup.bash; 
                g++ ~/cws/abb_dual_arm/src/abb_dual_arm/abb_irb120_support/src/identifyCorner.cpp -o identifyCorner \`pkg-config --cflags --libs opencv4\`; 
                ./identifyCorner; 
                exec bash'" &

# Wait for corner identification to complete
echo "Press any key once corner identification is complete..."
read -n 1 -s

# Launch the frames in a new Terminator window
terminator -e "bash -c 'source ~/cws/abb_dual_arm/devel/setup.bash; 
                roslaunch abb_irb120_support camera_tf.launch; 
                exec bash'" &

# Wait for frame to come up
echo "Press any key once frames are launched..."
read -n 1 -s

# Launch final move group in a new Terminator window
terminator -e "bash -c 'source ~/cws/abb_dual_arm/devel/setup.bash; 
                roslaunch abb_irb120_support abb_irb120_cartesian_planning.launch; 
                exec bash'" &