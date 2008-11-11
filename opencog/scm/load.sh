#! /bin/sh
#
# load.sh
#
# Initialize the scheme system with assorted basic definitions.
#

HOST=localhost
PORT=17001

cat type_constructors.scm |netcat -q0 $HOST $PORT
sleep 2;
cat utilities.scm         |netcat -q0 $HOST $PORT
sleep 2;
cat cgw-wire.scm          |netcat -q0 $HOST $PORT
sleep 2;
cat cgw-simple.scm        |netcat -q0 $HOST $PORT
sleep 2;
cat cgw-fan.scm           |netcat -q0 $HOST $PORT
sleep 2;
cat cgw-filter.scm        |netcat -q0 $HOST $PORT
sleep 2;
cat cgw-bidi.scm          |netcat -q0 $HOST $PORT
sleep 2;
cat cgw-predicate.scm     |netcat -q0 $HOST $PORT

# Session doesn't work over telnet.
# echo -e "scm\n(use-modules (ice-9 session))\n.\nexit\n"  |netcat -q0 $HOST $PORT
