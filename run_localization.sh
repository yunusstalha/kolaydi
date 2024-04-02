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
create_tmux_session "joy" "rosrun joy joy_node "
create_tmux_session "rplidar" "roslaunch rplidar_ros rplidar_a1.launch"
sleep 2
create_tmux_session "tf_publisher" "python3 src/lidar_pkg/src/scripts/localization_tf_pub.py"
sleep 1
create_tmux_session "odom" "roslaunch lidar_pkg odom.launch"
sleep 1
create_tmux_session "map" "rosrun map_server map_server ogrids/map.yaml"
sleep 1
create_tmux_session "amcl" "roslaunch lidar_pkg amcl_loc.launch"
sleep 1 
create_tmux_session "rviz" "rviz"

# Create main session and split into three panes
tmux new-session -s main -n script -d

# Source ROS in each pane and run different commands
tmux send-keys -t main "source devel/setup.bash" C-m

# Tmux configuration
tmux set -g pane-border-status top
tmux set -g window-active-style 'fg=colour250,bg=black'
tmux set -w window-status-separator '|'
tmux set -g pane-active-border-style "bg=default fg=green"
tmux set -g status-left-length 20
tmux set -g mouse

# Attach to main session
tmux attach -t main
