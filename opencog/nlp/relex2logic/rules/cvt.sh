#! /bin/bash

FILES=*.scm

for f in $FILES; do
	echo "Processing $f..."
	cat $f | ./cvt-type.pl > xxx
	mv xxx $f
done
