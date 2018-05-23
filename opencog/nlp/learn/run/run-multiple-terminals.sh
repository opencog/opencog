#! /bin/bash
#
# run-multiple-terminals.sh
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

args=("$@")

# Gets mode of counter for the cogserver
if [ "$1" = "pairs" ]
then
   launcher=launch-pair-count.scm
   port_ini=17000
elif [ "$1" = "mst" ]
then
   launcher=launch-mst-parser.scm
   port_ini=19000
else
   echo "Usage: ./run-multiple-terminals.sh <mode> <language> <db_name> [<username>] [<password>]"
   echo "<mode> must be either pairs or mst"
   exit 0
fi

# Assign port value according to language
case $2 in
   en)
      port_end=5
      ;;
   fr)
      port_end=3
      ;;
   lt)
      port_end=2
      ;;
   pl)
      port_end=4
      ;;
   yue)
      port_end=6
      ;;
   zh)
      port_end=7
      ;;
   *)
      echo "Usage: ./run-multiple-terminals.sh <mode> <language> <db_name> [<username>] [<password>]"
      echo "<language> must be one of: en, fr, lt, pl, yue, zh"
      exit 0
esac
PORT=$(($port_ini + $port_end))

# Start multiple sessions (use byobu so that the scroll bars actually work)
# Call launch-??.scm to start cogserver
byobu new-session -d -n 'cntl' '$SHELL'
case $# in
   3)
      byobu new-window -n 'cogsrv' "nice guile -l $launcher -- --lang $2 --db $3; $SHELL"
      ;;
   4)
      byobu new-window -n 'cogsrv' "nice guile -l $launcher -- --lang $2 --db $3 --user $4; $SHELL"
      ;;
   *)
      byobu new-window -n 'cogsrv' "nice guile -l $launcher -- --lang $2 --db $3 --user $4 --password $5; $SHELL"
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
