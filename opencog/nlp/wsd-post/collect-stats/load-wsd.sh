#! /bin/sh
#
# load-wsd.sh
#
# Load code for collecting WSD statistics, via batch-processing of
# files.
#

HOST=localhost
PORT=17001

cat stats-collection.scm |netcat -q60 $HOST $PORT > /dev/null
cat wsd-process.scm      |netcat -q60 $HOST $PORT > /dev/null
cat A.scm                |netcat -q60 $HOST $PORT > /dev/null

