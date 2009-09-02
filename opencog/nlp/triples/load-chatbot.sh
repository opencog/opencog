#! /bin/sh
#
# Load scm bits and pieces needed for triples handling in the chatbot. 
#

HOST=localhost
PORT=17004

# Load rules that generate the triples
cat rules.txt | ./rules-to-implications.pl frame-rule | netcat -q60 $HOST $PORT > /dev/null

# Experimental question-piepine
cat question-pipeline.txt | ./rules-to-implications.pl quest-rule |netcat -q60 $HOST $PORT > /dev/null
