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
# Note: must be run from build directory and path to relex
# distribution updated (RelEx script opencog-server.sh must 
# also be properly configured).

relex_path=/home/joel/src/relex
bot_name=cogita-$RANDOM

gnome-terminal \
  --hide-menubar \
  --tab --title "CogServer" \ #--working-directory= \
  --command="./opencog/server/cogserver -c ../lib/opencog-chatbot.conf" \
  --tab --title "RelEx" \
  --working-directory=${relex_path} --command="./opencog-server.sh" \
  --active --tab --title "Cogita chatbot" \ #--working-directory=. \
  --command="./opencog/nlp/chatbot/cogita -n ${bot_name} -c opencog-sandbox" \
  &

sleep 2
# load axioms for chatting into cogserver
cd ../opencog/nlp/triples/
./load-chatbot.sh
cd -


