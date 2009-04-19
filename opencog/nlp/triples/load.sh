#! /bin/sh
#
# Load all required scm bits and pieces in order to extract triples
#

# Load the preposition dictionary
cat prep-maps.scm | netcat -q1 localhost 17003 > /dev/null
cat preps.txt | ./preplist-to-atoms.pl | netcat -q2 localhost 17003 > /dev/null

# Load rules that generate the triples
cat prep-rules.txt | ./rules-to-implications.pl prep-rule | netcat -q3 localhost 17003 > /dev/null
cat rules.txt | ./rules-to-implications.pl frame-rule | netcat -q3 localhost 17003 > /dev/null

cat collect-stats.scm | netcat -q2 localhost 17003 > /dev/null
