#! /bin/bash
#
# main.sh
#
# Start the cogserver, run stuff

# Without this, some ros messages seem to run astray.
export ROS_IP=127.0.0.1

# Use byobu so that the scroll bars actually work
byobu new-session -d -n 'roscore' 'roscore; $SHELL'
sleep 4;

tmux new-window -n 'cogserver' 'cogserver; $SHELL'

# Load data into the CogServer
sleep 3
echo -e "py\n" | cat - atomic.py |netcat localhost 17001
cat universal-fsm.scm |netcat localhost 17001

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach
echo "Started"
