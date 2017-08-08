#!/bin/bash
#
# Download Chinese book texts
#
# for f in {1..100000} do
for f in `seq 1 2`; do
	let NUM=2000+$f
	wget http://www.ixdzs.com/down/$NUM"_1"
done
# mkdir somefolder
# unzip "*" -d directory/somefolder
# cd somefolder
# ls | cat -n | while read n f; do mv "$f" "$n.txt";done
