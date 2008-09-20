#! /bin/sh
#
# load.sh
#
# Initialize the scheme system with assorted basic definitions.
#

HOST=localhost
PORT=17001

cat type_constructors.scm |netcat $HOST $PORT
cat utilities.scm |netcat $HOST $PORT
cat wiring.scm |netcat $HOST $PORT
