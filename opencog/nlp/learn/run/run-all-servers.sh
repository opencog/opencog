#! /bin/bash
#
# run-all-servers.sh
#
# All-in-one script to start all servers on the local machine
# This runs tmux with byobu to multiplex multiple terminals;
# use F3 and F4 to swtich to the other terminals.
#

# Work around an lxc-attach bug.
if [[ `tty` == "not a tty" ]]
then
	script -c $0 /dev/null
	exit 0
fi

export LD_LIBRARY_PATH=/usr/local/lib/opencog/modules

# Use byobu so that the scroll bars actually work
byobu new-session -d -n 'cntl' '$SHELL'
byobu new-window -n 'cogsrv' 'nice guile -l pair-count-en.scm; $SHELL'
sleep 2;

# Relex "any" language
# Not using relex any longer.
# tmux new-window -n 'relex' './relex-server-any.sh; $SHELL'

# Telnet window
tmux new-window -n 'telnt' 'rlwrap telnet localhost 17005; $SHELL'

# Parse
# ./wiki-ss-en.sh
tmux new-window -n 'parse' '$SHELL'

# Spare
tmux new-window -n 'spare' '$SHELL'

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach

echo "Started"
