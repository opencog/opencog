#! /bin/sh
#
# load-nlp.sh
#
# Initialize the scheme system with assorted nlp utilities
#

HOST=localhost
PORT=17001

cat nlp-utils.scm |netcat $HOST $PORT
cat disjunct-list.scm |netcat $HOST $PORT
cat stats-collection.scm |netcat $HOST $PORT

