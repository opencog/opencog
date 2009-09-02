#! /bin/sh
#
# Load all required scm bits and pieces in order to extract triples
#

HOST=localhost
PORT=17003

# Load rules that generate the triples
cat rules.txt | ./rules-to-implications.pl frame-rule | netcat -q60 $HOST $PORT > /dev/null
cat question-tags.txt | ./rules-to-implications.pl question-tag | netcat -q60 $HOST $PORT > /dev/null

