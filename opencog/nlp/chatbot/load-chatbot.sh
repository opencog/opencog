#! /bin/sh
#
# load-chatbot.sh
#
# Load code for chatbot
#

HOST=localhost
PORT=17004

echo "scm" | cat - chat-interface.scm     |netcat -q60 $HOST $PORT > /dev/null

