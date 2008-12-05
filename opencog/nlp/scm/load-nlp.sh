#! /bin/sh
#
# load-nlp.sh
#
# Initialize the scheme system with assorted nlp utilities
#

HOST=localhost
PORT=17001

cat type-definitions.scm |netcat -q0 $HOST $PORT
sleep 1;
cat nlp-utils.scm        |netcat -q0 $HOST $PORT
sleep 1;
cat disjunct-list.scm    |netcat -q0 $HOST $PORT
sleep 1;
cat stats-collection.scm |netcat -q0 $HOST $PORT
sleep 2;
cat file-process.scm     |netcat -q0 $HOST $PORT

