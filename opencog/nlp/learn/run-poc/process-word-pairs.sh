#!/bin/bash
#
# process-word-pairs.sh <mode> <language>
#
# Support script for computing mi and fetching word pairs.
#
# Example usage:
#    ./process-word-pairs.sh cmi en
#

if [ $# -ne 2 ]
then 
  echo "Usage: ./process-word-pairs.sh <mode> <language>"
  exit 0
fi

# Set up assorted constants needed to run.
cnt_mode="clique-dist"
source ./config/params.txt
source ./config/det-port-num.sh $1 $2

# Gets processing instructions for the cogserver
case $1 in
   cmi)
      module="compute-mi.scm"
      func="comp-mi"
      ;;
   mst)
      module="fetch-word-pairs.scm"
      func="fetch-wp"
      ;;
   *)
      echo "Usage: ./process-word-pairs.sh <mode> <language>"
      echo "<mode> must be either cmi or mst"
      exit 0
esac

# Punt if the cogserver has crashed: use netcat to ping it.
haveping=`echo foo | nc -N localhost $PORT`
if [[ $? -ne 0 ]] ; then
	exit 1
fi

# Submit instruction to the cogserver
echo -e "(load \"$module\")" | nc -N localhost $PORT
echo -e "($func \"$cnt_mode\")" | nc -N localhost $PORT
