#! /bin/sh
#
# load.sh
#
# Load code for MIT ConceptNet batch processing
#

HOST=localhost
PORT=17003

cat seme-process.scm     |netcat -q60 $HOST $PORT > /dev/null
cat mit-conceptnet.scm   |netcat -q60 $HOST $PORT > /dev/null
cat config.scm           |netcat -q60 $HOST $PORT > /dev/null

