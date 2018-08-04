#! /bin/bash
#
# run-server-parse.sh
#
# Start cogserver on the local machine
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

if [ $# -lt 2 ]
then 
  echo "Usage: ./run-server-parse.sh <language> <db_name> [<username>] [<password>]"
  exit 0
fi

args=("$@")
# Use byobu so that the scroll bars actually work
byobu new-session -d -n 'cntl' '$SHELL'
if [ $# -eq 2 ]
then
	byobu new-window -n 'cogsrv' "nice guile -l observe-launch.scm -- --lang $1 --db $2 ; $SHELL"
elif [ $# -eq 3 ]
then
	byobu new-window -n 'cogsrv' "nice guile -l observe-launch.scm -- --lang $1 --db $2 --user $3; $SHELL"
elif [ $# -eq 4 ]
then
	byobu new-window -n 'cogsrv' "nice guile -l observe-launch.scm -- --lang $1 --db $2 --user $3 --password $4; $SHELL"
fi
sleep 2;

# Relex "any" language
# Not using relex any longer.
# tmux new-window -n 'relex' './relex-server-any.sh; $SHELL'

# Get port number according to mode and language
source ./config/det-port-num.sh pairs $1

# Telnet window
tmux new-window -n 'telnet' "rlwrap telnet localhost $PORT; $SHELL"

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
