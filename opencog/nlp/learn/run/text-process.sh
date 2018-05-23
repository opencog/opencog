#!/bin/bash
#
# Batch word-pair-counting/MST-parsing script for a given language
# Loop over all the files in 'beta-pages'/'gamma-pages' directory,
# sentence-split them and submit them for word-pair-counting/parsing.
#

if [ $# -ne 2 ]
then 
  echo "Usage: ./text-process.sh <mode> <language>"
  exit 0
fi


# Gets mode of counter for the cogserver
if [ "$1" = "pairs" ]
then
   port_ini=17000
   directory=beta-pages
elif [[ ("$1" = "mst") || ("$1" = "mst-extra") ]]
then
   port_ini=19000
   directory=gamma-pages
else
   echo "Usage: ./text-process.sh <mode> <language>"
   echo "<mode> must be either pairs or mst"
   exit 0
fi


# Assign port value according to language
case $2 in
   en)
      port_end=5
      ;;
   fr)
      port_end=3
      ;;
   lt)
      port_end=2
      ;;
   pl)
      port_end=4
      ;;
   yue)
      port_end=6
      ;;
   zh)
      port_end=7
      ;;
   *)
      echo "Usage: ./text-process.sh <mode> <language>"
      echo "<language> must be one of: en, fr, lt, pl, yue, zh"
      exit 0
esac
PORT=$(($port_ini + $port_end))

time find $directory -type f -exec ./process-one.sh $1 $2 {} localhost $PORT \;

