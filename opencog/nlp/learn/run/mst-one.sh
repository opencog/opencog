#!/bin/bash
#
# mst-one.sh <lang> <filename> <cogserver-host> <cogserver-port>
#
# Support script for batch parsing of wikipedia articles.
# Sentence-split one file, submit it, via perl script, to the parser.
# When done, move the file over to a 'finished' directory.
#
# Example usage:
#    ./mst-one.sh en Barbara localhost 17001
#

# Set up assorted constants needed to run.
lang=$1
filename="$2"
# coghost="localhost"
# cogport=17002
coghost="$3"
cogport=$4

splitter=/home/ubuntu/src/relex/src/split-sentences/split-sentences.pl
splitter=/usr/local/bin/split-sentences.pl
splitter=./split-sentences.pl

splitdir=split-articles
subdir=mst-articles
observe="observe-mst"

# Punt if the cogserver has crashed. The grep is looking for the
# uniquely-named config file.
# haveserver=`ps aux |grep cogserver |grep opencog-$lang`
# if [[ -z "$haveserver" ]] ; then
# 	exit 1
# fi
# Alternate cogserver test: use netcat to ping it.
haveping=`echo foo | nc -N $coghost $cogport`
if [[ $? -ne 0 ]] ; then
	exit 1
fi


# Split the filename into two parts
base=`echo $filename | cut -d \/ -f 1`
rest=`echo $filename | cut -d \/ -f 2-6`

echo "MST-Processing file >>>$rest<<<"

# Create directories if missing
mkdir -p $(dirname "$splitdir/$rest")
mkdir -p $(dirname "$subdir/$rest")

# Sentence split the article itself
cat "$filename" | $splitter -l $lang >  "$splitdir/$rest"

# Submit the split article
cat "$splitdir/$rest" | ./submit-one.pl $coghost $cogport $observe

# Punt if the cogserver has crashed (second test,
# before doing the mv and rm below)
# haveserver=`ps aux |grep cogserver |grep opencog-$lang`
# if [[ -z "$haveserver" ]] ; then
# 	exit 1
# fi
haveping=`echo foo | nc -N $coghost $cogport`
if [[ $? -ne 0 ]] ; then
	exit 1
fi

# Move article to the done-queue
mv "$splitdir/$rest" "$subdir/$rest"
rm "$base/$rest"
