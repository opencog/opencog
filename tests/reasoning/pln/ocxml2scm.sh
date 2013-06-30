#!/bin/bash

# Tacky script to convert OpenCog XML files to Scheme format. Only works
# if you remove the NLP-related scm files from SCM_PRELOAD in opencog.conf
# (otherwise a lot of extra atoms will be included).
# Use after starting the cogserver.

./opencog/server/cogserver -c ../lib/opencog.conf &
sleep 5;
export SCM_NAME=`echo $1|sed s/.xml/.scm/`
echo $SCM_NAME
#echo "pln load-axioms $1
echo "load $1
scm
(cog-delete (AnchorNode \"# New Parsed Sentence\"))
(export-all-atoms \"$SCM_NAME\")
.
exit" | nc localhost 17001

sleep 5; # lest that hasn't finished by the time the next cogserver gets started

killall cogserver
sleep 5;
