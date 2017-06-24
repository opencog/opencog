#! /bin/bash

wget $1
ebook-convert *.epub foo.txt
./chapters-epub.sh foo.txt $2

mv $2* split-books
mv *epub whole-books
rm foo.txt
