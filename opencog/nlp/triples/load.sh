#! /bin/sh
#
# Load all required scm bits and pieces in order to extract triples
#

# Load the preposition dictionary
cat prep-maps.scm | netcat -q0 localhost 17003

# Load add rules
cat rules.txt | ./rules-to-implications.pl | netcat -q0 localhost 17003

# Load some previously parsed sentences.
cat example-sentences.scm | netcat -q0 localhost 17003

