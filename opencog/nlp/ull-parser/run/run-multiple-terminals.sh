#! /bin/bash
#
# run-multiple-terminals.sh <mode> <language> <db_name> [<username>] [<password>]
#
# Start cogserver on the local machine
# This runs tmux with byobu to multiplex several terminals;
# Use F3 and F4 to swtich to the other terminals.
#

# Work around an lxc-attach bug.
if [[ `tty` == "not a tty" ]]
then
	script -c $0 /dev/null
	exit 0
fi

export LD_LIBRARY_PATH=/usr/local/lib/opencog/modules

if [ $# -lt 3 ]
then 
  echo "Usage: ./run-multiple-terminals.sh <mode> <language> <db_name> [<username>] [<password>]"
  exit 0
fi

# Get port number according to mode and language
source ./config/det-port-num.sh $1 $2
launcher=launch-cogserver.scm

# Start multiple sessions (use byobu so that the scroll bars actually work)
# Call launch-??.scm to start cogserver
byobu new-session -d -n 'cntl' '$SHELL'
case $# in
   3)
      byobu new-window -n 'cogsrv' "nice guile -l $launcher -- --mode $1 --lang $2 --db $3; $SHELL"
      ;;
   4)
      byobu new-window -n 'cogsrv' "nice guile -l $launcher -- --mode $1 --lang $2 --db $3 --user $4; $SHELL"
      ;;
   *)
      byobu new-window -n 'cogsrv' "nice guile -l $launcher -- --mode $1 --lang $2 --db $3 --user $4 --password $5; $SHELL"
      ;;
esac
sleep 2;


# Telnet window
tmux new-window -n 'telnet' "rlwrap telnet localhost $PORT; $SHELL"

# Parse
# ./text-process.sh
tmux new-window -n 'parse' '$SHELL'

# Spare
tmux new-window -n 'spare' '$SHELL'

# Fix the annoying byobu display
echo "tmux_left=\"session\"" > $HOME/.byobu/status
echo "tmux_right=\"load_average disk_io date time\"" >> $HOME/.byobu/status
tmux attach

echo "Started"
