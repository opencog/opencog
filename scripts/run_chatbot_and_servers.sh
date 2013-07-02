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

branch_dir=/home/joel/src/opencog/trunk
build_dir=${branch_dir}/bin

relex_path=/home/joel/src/relex
bot_name=cogita-$RANDOM

old_dir=`pwd`
cd ${build_dir}
gnome-terminal \
  --hide-menubar \
  --tab --title "CogServer" \ #--working-directory= \
  --command="./opencog/server/cogserver -c ../lib/opencog-chatbot.conf -DLOG_TO_STDOUT=TRUE" \
  --tab --title "RelEx" \
  --working-directory=${relex_path} --command="./opencog-server.sh" \
  --active --tab --title "Cogita chatbot" \ #--working-directory=. \
  --command="./opencog/nlp/chatbot/cogita -n ${bot_name} -c opencog-sandbox" \
  &

sleep 2
# load axioms for chatting into cogserver
cd ${build_dir}/opencog/nlp/triples/
./load-chatbot.sh
cd ${old_dir}


