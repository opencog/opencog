#!/bin/bash
#
# Support script for batch parsing of wikipedia data
# Sentence-split one file, submit it, via perl script,
# to the parser. When done, move the file over to a 'finished'
# directory.
#
# Set up assorted cnstants needed to run.

lang=$1
filename="$2"

splitter=/home/lmilambi/relex/src/split-sentences/split-sentences.pl
splitdir=split-articles
subdir=submitted-articles

# Punt if the cogserver has crashed
haveserver=`ps aux |grep cogserver |grep opencog-$lang`
if [[ -z "$haveserver" ]] ; then
	exit 1
fi
haveserver=`ps aux |grep relex |grep linkgram`
if [[ -z "$haveserver" ]] ; then
	exit 1
fi


# Split the filename into two parts
base=`echo $filename | cut -d \/ -f 1`
rest=`echo $filename | cut -d \/ -f 2-6`

echo "Processing file $rest"

# Sentence split the article itself
cat "$filename" | $splitter -l $lang >  "$splitdir/$rest" 

# Submit the split article
cat "$splitdir/$rest" | ./submit-one.pl 
echo "blah blah blah $splitdir/$rest"

# Punt if the cogserver has crashed (second test,
# before doing teh mv and rm below)
haveserver=`ps aux |grep cogserver |grep opencog-tl`
if [[ -z "$haveserver" ]] ; then
	exit 1
fi
haveserver=`ps aux |grep relex |grep linkgram`
if [[ -z "$haveserver" ]] ; then
	exit 1
fi

# Move article to the done-queue
mv "$splitdir/$rest" "$subdir/$rest"
rm "$base/$rest"
