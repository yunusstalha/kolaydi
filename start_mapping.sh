#!/bin/bash

# Name of the tmux session
SESSION="ROS_Workspace"

# Create a new tmux session and detach from it
tmux new-session -d -s $SESSION

# Window 1: Core
tmux rename-window -t 0 'Core'
tmux send-keys 'source ~/.bashrc' C-m
tmux send-keys 'source /opt/ros/noetic/setup.bash' C-m
tmux send-keys 'source ~/git/kolaydi/devel/setup.bash' C-m



# Split Window into three panes
# Motor control
tmux split-window -h
tmux select-pane -t 1
tmux send-keys 'source ~/.bashrc' C-m
tmux send-keys 'source /opt/ros/noetic/setup.bash' C-m
tmux send-keys 'source ~/git/kolaydi/devel/setup.bash' C-m
tmux send-keys 'rosrun robot_control motor_control.py' C-m

# Arduino, servo control
tmux split-window -v
tmux select-pane -t 2
tmux send-keys 'source ~/.bashrc' C-m
tmux send-keys 'source /opt/ros/noetic/setup.bash' C-m
tmux send-keys 'source ~/git/kolaydi/devel/setup.bash' C-m
tmux send-keys 'rosrun rosserial_python serial_node.py /dev/ttyUSB0' C-m

# TF publisher
tmux split-window -h
tmux select-pane -t 3
tmux send-keys 'source ~/.bashrc' C-m
tmux send-keys 'source /opt/ros/noetic/setup.bash' C-m
tmux send-keys 'source ~/git/kolaydi/devel/setup.bash' C-m
tmux send-keys 'python3 ~/git/kolaydi/src/lidar_pkg/scripts/mapping_tf_broadcaster_node.py' C-m
tmux send-keys 'p' C-m

# Laser to PCL
tmux split-window -h
tmux select-pane -t 4
tmux send-keys 'source ~/.bashrc' C-m
tmux send-keys 'source /opt/ros/noetic/setup.bash' C-m
tmux send-keys 'source ~/git/kolaydi/devel/setup.bash' C-m
tmux send-keys 'python3 ~/git/kolaydi/src/lidar_pkg/scripts/laser_scan_to_point_cloud_node.py' C-m
tmux send-keys 'p' C-m

# Select the first pane for LiDAR init
tmux select-pane -t 0
tmux send-keys 'roslaunch rplidar_ros rplidar_a1.launch' C-m

# Window 2: Development
tmux new-window -t $SESSION -n 'Dev'
tmux send-keys 'source ~/.bashrc' C-m
tmux send-keys 'source /opt/ros/noetic/setup.bash' C-m
tmux send-keys 'source ~/git/kolaydi/devel/setup.bash' C-m

# Attach to the tmux session
tmux attach-session -t $SESSION
