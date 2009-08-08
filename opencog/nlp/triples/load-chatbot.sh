#! /bin/sh
#
# Load scm bits and pieces needed for triples handling in the chatbot. 
#

HOST=localhost
PORT=17004

# Load the preposition dictionary
cat prep-maps.scm | netcat -q60 $HOST $PORT > /dev/null
cat preps.txt | ./preplist-to-atoms.pl | netcat -q60 $HOST $PORT > /dev/null

# Load rules that generate the triples
cat prep-rules.txt | ./rules-to-implications.pl prep-rule | netcat -q60 $HOST $PORT > /dev/null
cat rules.txt | ./rules-to-implications.pl frame-rule | netcat -q60 $HOST $PORT > /dev/null

