#! /bin/sh
#
# load.sh
#
# Load code for chatbot
#

HOST=localhost
PORT=17004

cat seme-process.scm     |netcat -q60 $HOST $PORT > /dev/null

