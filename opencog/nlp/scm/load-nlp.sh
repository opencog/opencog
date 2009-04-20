#! /bin/sh
#
# load-nlp.sh
#
# Initialize the scheme system with assorted nlp utilities
#

HOST=localhost
PORT=17001

# echo "scm" | cat - type-definitions.scm |netcat -q2 $HOST $PORT
# echo "scm" | cat - nlp-utils.scm        |netcat -q2 $HOST $PORT
cat disjunct-list.scm    |netcat -q2 $HOST $PORT
cat stats-collection.scm |netcat -q2 $HOST $PORT
cat wsd-process.scm      |netcat -q2 $HOST $PORT
cat A.scm                |netcat -q2 $HOST $PORT

