#!/bin/bash

# Setup the default ROS Noetic environment
source /opt/ros/noetic/setup.bash
source ~/.bashrc
export ROS_PACKAGE_PATH="~/git/kolaydi:$ROS_PACKAGE_PATH"

# Function to create a new tmux session and run a command if it does not already exist
create_tmux_session() {
    SESSIONNAME=$1
    COMMAND=$2

    # Check if the session exists, create if it does not
    tmux has-session -t $SESSIONNAME &> /dev/null
    if [ $? != 0 ]; then
        tmux new-session -s $SESSIONNAME -n script -d
        tmux send-keys -t $SESSIONNAME:script "source /opt/ros/noetic/setup.bash" C-m
        tmux send-keys -t $SESSIONNAME:script "source ~/git/kolaydi/devel/setup.bash" C-m
        tmux send-keys -t $SESSIONNAME:script "$COMMAND" C-m
    fi
}

# Create the main session and configure it before creating other sessions
tmux new-session -s main -n script -d
tmux send-keys -t main:script "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys -t main:script "source ~/git/kolaydi/devel/setup.bash" C-m

# Set tmux configurations for the main session
tmux set -g pane-border-status top
tmux set -g window-active-style 'fg=colour250,bg=black'
tmux set -w window-status-separator '|'
tmux set -g pane-active-border-style "bg=default fg=green"
tmux set -g status-left-length 20
tmux set -g mouse on

# Create other tmux sessions for various tasks
create_tmux_session "rplidar" "roslaunch rplidar_ros rplidar_a1.launch"
sleep 2
create_tmux_session "tf_publisher" "python3 ~/git/kolaydi/src/lidar_pkg/scripts/localization_tf_pub.py"
sleep 3
create_tmux_session "odom" "roslaunch lidar_pkg odom.launch"
sleep 3
create_tmux_session "amcl" "roslaunch lidar_pkg amcl_localization.launch"

# Attach to the main session
tmux attach -t main
