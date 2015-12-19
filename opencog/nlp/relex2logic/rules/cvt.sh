#! /bin/bash

FILES=*.scm

for f in $FILES; do
	echo "Processing $f..."
	cat $f | ./cvt-word.pl > xxx
	mv xxx $f
done
