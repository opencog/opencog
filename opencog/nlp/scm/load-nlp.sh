#! /bin/sh
#
# load-nlp.sh
#
# Initialize the scheme system with assorted nlp utilities
#

HOST=localhost
PORT=17001

cat type-definitions.scm |netcat -q0 $HOST $PORT
cat nlp-utils.scm        |netcat -q0 $HOST $PORT
cat disjunct-list.scm    |netcat -q0 $HOST $PORT
cat stats-collection.scm |netcat -q0 $HOST $PORT
cat file-process.scm     |netcat -q0 $HOST $PORT
cat A.scm                |netcat -q0 $HOST $PORT

