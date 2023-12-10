#!/bin/bash

# Setup the default ROS Noetic environment
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH="/home/kovan3/git/kolaydi:$ROS_PACKAGE_PATH"

# Function to create a new tmux session and run a command
create_tmux_session() {
    SESSIONNAME=$1
    COMMAND=$2

    tmux has-session -t $SESSIONNAME &> /dev/null
    if [ $? != 0 ]; then
        tmux new-session -s $SESSIONNAME -n script -d
        tmux send-keys -t $SESSIONNAME "source devel/setup.bash" C-m
        tmux send-keys -t $SESSIONNAME "$COMMAND" C-m
    fi
}

# Create tmux sessions for various tasks
create_tmux_session "rplidar" "roslaunch rplidar_ros rplidar_a1.launch"
sleep 3
create_tmux_session "tf_publisher" "python3 src/lidar_pkg/src/scripts/mapping_tf_publisher.py"
sleep 1
create_tmux_session "arduino" "rosrun rosserial_python serial_node.py /dev/ttyUSB1"
sleep 1
create_tmux_session "octo" "roslaunch lidar_pkg mapping.launch"
sleep 1 
create_tmux_session "rviz" "rviz"

# Create main session and split into three panes
tmux new-session -s main -n script -d
tmux split-window -v -t main
tmux split-window -h -t main

# Source ROS in each pane and run different commands
tmux send-keys -t main.0 "source devel/setup.bash" C-m "python3 src/lidar_pkg/src/scripts/laserscan2pcl.py" C-m
tmux send-keys -t main.1 "source devel/setup.bash" C-m
tmux send-keys -t main.2 "source devel/setup.bash" C-m "rqt" C-m

# Tmux configuration
tmux set -g pane-border-status top
tmux set -g window-active-style 'fg=colour250,bg=black'
tmux set -w window-status-separator '|'
tmux set -g pane-active-border-style "bg=default fg=green"
tmux set -g status-left-length 20
tmux set -g mouse

# Attach to main session
tmux attach -t main
