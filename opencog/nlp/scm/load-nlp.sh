#! /bin/sh
#
# load-nlp.sh
#
# Initialize the scheme system with assorted nlp utilities
#

HOST=localhost
PORT=17001

# echo "scm" | cat - type-definitions.scm |netcat -q60 $HOST $PORT
# echo "scm" | cat - nlp-utils.scm        |netcat -q60 $HOST $PORT
# cat disjunct-list.scm    |netcat -q60 $HOST $PORT > /dev/null
cat stats-collection.scm |netcat -q60 $HOST $PORT > /dev/null
cat wsd-process.scm      |netcat -q60 $HOST $PORT > /dev/null
cat A.scm                |netcat -q60 $HOST $PORT > /dev/null

