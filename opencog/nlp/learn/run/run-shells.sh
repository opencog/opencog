#! /bin/bash
#
# run-shells.sh
#
# Run tmux with byobu to multiplex multiple terminals; start the
# CogServer in one terminal, and suggest which processes to run
# in others.
#
# Use F3 and F4 to switch to the other terminals.
#

# Work around an lxc-attach bug.
if [[ `tty` == "not a tty" ]]
then
	script -c $0 /dev/null
	exit 0
fi

export LD_LIBRARY_PATH=/usr/local/lib/opencog/modules

# Use byobu so that the scroll bars actually work
byobu new-session -d -n 'cntl' \
	'echo -e "\nControl shell; you might want to run 'top' here.\n"; $SHELL'
byobu new-window -n 'cogsrv' 'nice guile -l pair-count-en.scm; $SHELL'
sleep 2;

# Telnet window
tmux new-window -n 'telnet' 'rlwrap telnet localhost 17005; $SHELL'

# Parse
tmux new-window -n 'submit' \
	'echo -e "\nYou might want to run ./pair-submit-en.sh here.\n"; $SHELL'

# Spare
tmux new-window -n 'spare' 'echo -e "\nSpare-use shell.\n"; $SHELL'

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach
