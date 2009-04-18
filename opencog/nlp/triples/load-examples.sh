#! /bin/sh
#
# Load example sentences for the example demo.
#
# Load some previously parsed sentences.
cat example-sentences.scm | netcat -q3 localhost 17003 > /dev/null

