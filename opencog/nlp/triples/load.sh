#! /bin/sh
#
# Load all required scm bits and pieces in order to extract triples
#

HOST=localhost
PORT=17003

# Load the preposition dictionary
cat prep-maps.scm | netcat -q60 $HOST $PORT > /dev/null
cat preps.txt | ./preplist-to-atoms.pl | netcat -q60 $HOST $PORT > /dev/null

# This doesn't work, because the full path is needed.
# echo -e "scm\n" \
# 	"(load-scm-from-file \"prep-maps.scm\")" \
# 	"(load-scm-from-file "preps.scm\")" \
# 	"\n.\nexit" | netcat -q60 $HOST $PORT

# Load rules that generate the triples
echo scm | cat - prep-rules.scm | netcat -q60 $HOST $PORT > /dev/null

cat rules.txt | ./rules-to-implications.pl frame-rule | netcat -q60 $HOST $PORT > /dev/null
cat question-tags.txt | ./rules-to-implications.pl question-tag | netcat -q60 $HOST $PORT > /dev/null

cat triples-pipeline.scm | netcat -q60 $HOST $PORT > /dev/null
