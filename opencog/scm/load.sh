#! /bin/sh
#
# load.sh
#
# Initialize the scheme system with assorted basic definitions.
#

HOST=localhost
PORT=17001

echo -e "scm\n(turn-on-debugging)\n.\nexit\n"  |netcat -q0 $HOST $PORT

echo "scm" | cat - type_constructors.scm  |netcat -q0 $HOST $PORT
echo "scm" | cat - utilities.scm          |netcat -q0 $HOST $PORT
echo "scm" | cat - file-utils.scm         |netcat -q0 $HOST $PORT

# The cgw code is not being used at this time. 
# cat cgw-wire.scm          |netcat -q0 $HOST $PORT
# cat cgw-simple.scm        |netcat -q0 $HOST $PORT
# cat cgw-fan.scm           |netcat -q0 $HOST $PORT
# cat cgw-filter.scm        |netcat -q0 $HOST $PORT
# cat cgw-bidi.scm          |netcat -q0 $HOST $PORT
# cat cgw-predicate.scm     |netcat -q0 $HOST $PORT

# Session doesn't work over telnet.
# echo -e "scm\n(use-modules (ice-9 session))\n.\nexit\n"  |netcat -q0 $HOST $PORT
