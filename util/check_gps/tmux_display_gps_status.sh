#!/bin/bash
#check if session already exists
tmux has-session -t neuroam 2>/dev/null
if [ $? != 0 ]; then
    #create a new session
    tmux new-session -d -s neuroam
fi

# Attach in a terminal window
gnome-terminal --full-screen -- bash -c "tmux attach-session -t neuroam; exec bash"

# run CGPS python
tmux send-keys -t neuroam "/home/neuroam/NeuROAM/util/check_gps/run_gps_ui.sh" C-m