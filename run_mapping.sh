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
create_tmux_session "roscore" "roscore"
sleep 1
echo "Octomap initialized" 
create_tmux_session "rviz" "rosrun rviz rviz -d ~/git/kolaydi/mapping.rviz"
sleep 1 
echo "Rviz initialized"

# Create main session and split into three panes
tmux new-session -s main -n script -d
tmux split-window -v -t main
tmux split-window -h -t main

# Source ROS in each pane and run different commands
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
