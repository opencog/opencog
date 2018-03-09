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

if [ $# -lt 2 ]
then 
  echo "Usage: ./run-server.sh <language> <db_name> [<username>] [<password>]"
  exit 0
fi

# Use byobu so that the scroll bars actually work
byobu new-session -d -n 'cntl' '$SHELL'
if [ $# -eq 2]
then
	byobu new-window -n 'cogsrv' 'nice guile -l observe-launch.scm -- --lang
	$1 --db $2 ; $SHELL'
elif [$# -eq 3]
then
	byobu new-window -n 'cogsrv' 'nice guile -l observe-launch.scm -- --lang
	$1 --db $2 --user $3; $SHELL'
elif [$# -eq 4]
then
	byobu new-window -n 'cogsrv' 'nice guile -l observe-launch.scm -- --lang
	$1 --db $2 --user $3 --password $4; $SHELL'
fi
sleep 2;

# Relex "any" language
# Not using relex any longer.
# tmux new-window -n 'relex' './relex-server-any.sh; $SHELL'

# Assign port value according to language
if [$1 -eq 'en']
then
  PORT=17005
elif [$1 -eq 'fr']
then
  PORT=17003
elif [$1 -eq 'lt']
then
  PORT=17002
elif [$1 -eq 'pl']
then
  PORT=17004
elif [$1 -eq 'yue']
then
  PORT=17006
elif [$1 -eq 'zh']
then
  PORT=17007
fi

# Telnet window
tmux new-window -n 'telnt' 'rlwrap telnet localhost $PORT; $SHELL'

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
