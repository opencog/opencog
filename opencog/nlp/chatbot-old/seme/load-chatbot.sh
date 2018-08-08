#! /bin/sh
#
# load-chatbot.sh
#
# Load code for chatbot
#

HOST=localhost
PORT=17004

echo "scm" | cat - seme-process.scm |netcat -q0 $HOST $PORT > /dev/null

