#! /bin/sh
#
# load-nlp.sh
#
# Initialize the scheme system with assorted nlp utilities
#

HOST=localhost
PORT=17001

cat type-definitions.scm |netcat $HOST $PORT
cat nlp-utils.scm |netcat $HOST $PORT
cat disjunct-list.scm |netcat $HOST $PORT
cat stats-collection.scm |netcat $HOST $PORT
cat file-process.scm |netcat $HOST $PORT

