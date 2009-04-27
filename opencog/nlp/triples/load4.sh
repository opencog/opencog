#! /bin/sh
#
# Load scm bits and pieces needed for triples handling in the chatbot. 
#

# Load the preposition dictionary
cat prep-maps.scm | netcat -q60 localhost 17004 > /dev/null
cat preps.txt | ./preplist-to-atoms.pl | netcat -q60 localhost 17004 > /dev/null

# Load rules that generate the triples
cat prep-rules.txt | ./rules-to-implications.pl prep-rule | netcat -q60 localhost 17004 > /dev/null
cat rules.txt | ./rules-to-implications.pl frame-rule | netcat -q60 localhost 17004 > /dev/null

