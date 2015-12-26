#!/bin/bash -e
## OpenCog chatbot script
#
# Copyright 2009 - Joel Pitt
#
# Script launches a gnome-terminal with three tabs:
# - CogServer
# - RelEx server
# - Cogita chatbot
#
# It also loads the CogServer with the appropriate
# guff for it to function.
# 
# Note: path to src and build directory and path to relex distribution need to
# be updated (RelEx script opencog-server.sh must also be properly configured).

# Store the path to the scripts directory (where this script is) to determine
# the path to the opencog source directory on the next line.
script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Strip the '/scripts' off the end of the path to get the path of the main opencog directory
branch_dir=${script_dir%/scripts}
build_dir=${branch_dir}/build

relex_path=/home/buck/Code/relex
bot_name=cogita-$RANDOM

old_dir=`pwd`
cd ${build_dir}
gnome-terminal \
  --maximize \
  --hide-menubar \
  --tab --title "CogServer" \ #--working-directory= \
  --command="./opencog/cogserver/server/cogserver -c ../lib/opencog-chatbot.conf -DLOG_TO_STDOUT=TRUE" \
  --tab --title "RelEx" \
  --working-directory=${relex_path} --command="./opencog-server.sh" \
  --active --tab --title "Cogita chatbot" \ #--working-directory=. \
  --command="./opencog/nlp/chatbot/cogita -n ${bot_name} -c opencog-sandbox" \
  &

sleep 2
# load axioms for chatting into cogserver
cd ${branch_dir}/opencog/nlp/chatbot
./load-chatbot.sh
cd ${old_dir}


