#! /bin/sh
#
# load.sh
#
# Load code for MIT ConceptNet processing
#

HOST=localhost
PORT=17003

cat mit-conceptnet.scm   |netcat -q60 $HOST $PORT > /dev/null
cat config.scm           |netcat -q60 $HOST $PORT > /dev/null

