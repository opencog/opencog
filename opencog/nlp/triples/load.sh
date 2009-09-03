#! /bin/sh
#
# Load all required scm bits and pieces in order to extract triples
#

HOST=localhost
PORT=17003

cat question-tags.txt | ./rules-to-implications.pl question-tag | netcat -q60 $HOST $PORT > /dev/null

