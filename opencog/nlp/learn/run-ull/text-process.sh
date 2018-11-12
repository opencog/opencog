#!/bin/bash
#
# text-process.sh <mode> <language>
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

# Gets directory for fetching corpora according to processing mode
case $1 in
   pairs)
      directory=beta-pages
      ;;
   mst)
      directory=gamma-pages
      ;;
   *)
      echo "Usage: ./text-process.sh <mode> <language>"
      echo "<mode> must be either pairs or mst"
      exit 0
esac

# Get port number according to mode and language
source ./config/det-port-num.sh $1 $2

# Process files in assigned directory
time find $directory -type f -exec ./process-one.sh $1 $2 {} localhost $PORT \;
