#! /bin/bash

FILES=*.scm

for f in $FILES; do
	echo "Processing $f..."
	# cat $f | ./cvt-type.pl > xxx
	# cat $f | ./cvt-word.pl > xxx
	# cat $f | ./cvt-eval.pl > xxx
	cat $f | ./lem.pl > xxx
	mv xxx $f
done
