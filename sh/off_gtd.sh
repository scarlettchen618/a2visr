#!/bin/bash

# Session name
SESSION_NAME="off_gtd"

# Check if the tmux session already exists
tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? != 0 ]; then
  # Create a new tmux session
  tmux new-session -d -s $SESSION_NAME

  # Step 1: Create 4 layout
  tmux split-window -h    # Split vertically to create two columns
  tmux split-window -v    # Split the left column into two rows
  tmux select-pane -t 0   # Go back to the first pane
  tmux split-window -v    # Split the right column into two rows

  # Step 3: Arrange the layout for better visualization
  tmux select-layout tiled  # Ensure the layout adjusts cleanly
  
  tmux send-keys -t $SESSION_NAME:0.1 'roslaunch mavros px4.launch'
  tmux send-keys -t $SESSION_NAME:0.2 'roslaunch serial_imu trans_imu_node.launch'
  tmux send-keys -t $SESSION_NAME:0.3 'roslaunch vrpn_client_ros sample.launch'
  tmux send-keys -t $SESSION_NAME:0.0 'roslaunch nlink_parser linktrack.launch'
fi

# Attach to the session
tmux attach-session -t $SESSION_NAME
