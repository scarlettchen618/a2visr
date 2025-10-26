#!/bin/bash

# Session name
SESSION_NAME="main"

# Check if the session already exists
tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? != 0 ]; then
  # Create a new tmux session
  tmux new-session -d -s $SESSION_NAME

  # Split into four panes
  tmux split-window -h    # Create a vertical split
  tmux split-window -v    # Create a horizontal split in the left pane
  tmux select-pane -t 0   # Go back to the first pane
  tmux split-window -v    # Create a horizontal split in the right pane

  # Step 2: Split the first pane vertically
  tmux select-pane -t 0   # Go back to the last pane (top-left)
  tmux split-window -h    # Split the first pane vertically

  # Step 3: Arrange the layout for better visualization
  tmux select-layout tiled  # Ensure the layout adjusts cleanly
  
  tmux send-keys -t $SESSION_NAME:0.2 'bash est_record.sh'
  tmux send-keys -t $SESSION_NAME:0.3 'roslaunch a2VISR a2VISR_multi_onboard.launch' 
  tmux send-keys -t $SESSION_NAME:0.4 'roslaunch offboard_a2VISR px4ctrl_circle_node.launch'
  tmux send-keys -t $SESSION_NAME:0.0 'rostopic echo /mavros/vision_pose/pose'
  tmux send-keys -t $SESSION_NAME:0.1 'rostopic echo /uav201/mocap/pos' 
  
fi

# Attach to the session
tmux attach-session -t $SESSION_NAME

