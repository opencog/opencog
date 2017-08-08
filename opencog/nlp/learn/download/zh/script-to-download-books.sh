#!/bin/bash
for f in {1..100000}
do
download_link="http://www.ixdzs.com/down/"$f
link="$download_link"_1
wget $link
f=$((f+1))
done
mkdir somefolder
unzip "*" -d directory/somefolder
cd somefolder
ls | cat -n | while read n f; do mv "$f" "$n.txt";done
