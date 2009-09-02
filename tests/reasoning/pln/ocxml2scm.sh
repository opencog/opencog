#!/bin/bash

# Tacky script to convert OpenCog XML files to Scheme format. Only works
# if you remove the NLP-related scm files from SCM_PRELOAD in opencog.conf
# (otherwise a lot of extra atoms will be included).
# Use after starting the cogserver.

export SCM_NAME=`echo $1|sed s/.xml/.scm/`
echo "load $1\nscm\n(export-all-atoms \"$SCM_NAME\")\n.\nexit" | nc localhost 17001


