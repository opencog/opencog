#! /bin/bash
# Download and split one file

wget $1

filename=`echo $1 | sed  s#^.*/## `
base=`echo $filename | sed  s#\.txt.*$s## `

echo  this $filename has basename $base-

cat $filename | sed -n '/START .* PROJECT GUTENBERG EBOOK/,$p' | \
	sed -n '1,/END .* PROJECT GUTENBERG EBOOK/p' > foo

grep -l 大 foo
if [ $? -ne 1 ]; then
   echo Found CJK characters, bailing
	exit
fi

grep -l Anmerkungen foo
if [ $? -ne 1 ]; then
   echo Suspected German book, bailing
	exit
fi

grep -l purezza foo
if [ $? -ne 1 ]; then
   echo Suspected Italian book, bailing
	exit
fi

./chapters.sh foo $base-
mv $filename whole-books
mv $base-* split-books
rm foo
