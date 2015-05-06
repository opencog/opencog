#! /bin/bash
#
# main.sh
#
# Start the cogserver, run stuff

# Use byobu so that the scroll bars actually work
byobu new-session -d -n 'roscore' 'roscore; $SHELL'
sleep 4;

tmux new-window -n 'cogserver' 'cogserver; $SHELL'

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach
echo "Started"
