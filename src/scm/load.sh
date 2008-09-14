#! /bin/sh
#
# load.sh
#
# Initialize the scheme system with assorted basic definitions.
#

cat type_constructors.scm |netcat localhost 17001
cat utilities.scm |netcat localhost 17001
