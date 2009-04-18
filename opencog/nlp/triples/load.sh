#! /bin/sh
#
# Load all required scm bits and pieces in order to extract triples
#

# Load the preposition dictionary
cat prep-maps.scm | netcat -q1 localhost 17003 > /dev/null
cat preps.txt | ./preplist-to-atoms.pl | netcat -q1 localhost 17003 > /dev/null

# Load add rules
cat rules.txt | ./rules-to-implications.pl | netcat -q2 localhost 17003 > /dev/null

# Load some previously parsed sentences.
cat example-sentences.scm | netcat -q3 localhost 17003 > /dev/null

