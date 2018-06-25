#!/bin/bash
#
# process-one.sh <mode> <lang> <filename> <cogserver-host> <cogserver-port>
#
# Support script for batch parsing of plain-text files.
# Sentence-split one file, submit it, via perl script, to the parser.
# When done, move the file over to a 'finished' directory.
#
# Example usage:
#    ./process-one.sh mst en Barbara localhost 17001
#

# Set up assorted constants needed to run.
lang=$2
filename="$3"
coghost="$4"
cogport=$5
splitter=./split-sentences.pl
splitdir=split-articles

#NOTE: commented out parameters that are not yet incorporated in this commit.
# Default parameter values 
# cnt_mode="clique-dist"
# cnt_reach=6
# mst_dist="#t"
exp_parses="#t"
source ./config/params.txt # overrides default values, if present

# Gets processing mode for the cogserver
case $1 in
   pairs)
      subdir=submitted-articles
      observe="observe-text"
      # params="$cnt_mode $cnt_reach"
      ;;
   mst)
      subdir=mst-articles
      observe="observe-mst"
      if [[ "$exp_parses" == "#t" ]]
          then observe="observe-mst-extra"
      fi
      # params="$cnt_mode $mst_dist"
      ;;
esac

# Punt if the cogserver has crashed: use netcat to ping it.
haveping=`echo foo | nc -N $coghost $cogport`
if [[ $? -ne 0 ]] ; then
	exit 1
fi

# Split the filename into two parts
base=`echo $filename | cut -d \/ -f 1`
rest=`echo $filename | cut -d \/ -f 2-6`

echo "Processing file >>>$rest<<<"

# Create directories if missing
mkdir -p $(dirname "$splitdir/$rest")
mkdir -p $(dirname "$subdir/$rest")

# Sentence split the article itself
cat "$filename" | $splitter -l $lang >  "$splitdir/$rest"

# Submit the split article
cat "$splitdir/$rest" | ./submit-one.pl $coghost $cogport $observe #$params

# Punt if the cogserver has crashed (second test, before doing the mv and rm below)
haveping=`echo foo | nc -N $coghost $cogport`
if [[ $? -ne 0 ]] ; then
	exit 1
fi

# Move article to the done-queue
mv "$splitdir/$rest" "$subdir/$rest"
rm "$base/$rest"
