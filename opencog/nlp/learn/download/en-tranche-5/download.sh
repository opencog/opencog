#! /bin/bash
#
# Download and prep a bunch of texts from project gutenberg.
# Each text will look something like this:
#
# ./process-gutenberg.sh http://www.gutenberg.org/files/53799/53799-0.txt
# Except that we will loop from 53499 to 53000

for i in `seq 0 499`;
do
	let ART=53000+$i;
	echo ./process-gutenberg.sh http://www.gutenberg.org/files/$ART/$ART-0.txt
	./process-gutenberg.sh http://www.gutenberg.org/files/$ART/$ART-0.txt
done
