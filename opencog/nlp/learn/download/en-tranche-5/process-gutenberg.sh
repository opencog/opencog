#! /bin/bash
# Download and split one file

wget $1

filename=`echo $1 | sed  s#^.*/## `
base=`echo $filename | sed  s#\.txt.*$s## `

echo  this $filename has basename $base-

cat $filename | sed -n '/START .* PROJECT GUTENBERG EBOOK/,$p' | \
	sed -n '1,/END .* PROJECT GUTENBERG EBOOK/p' > foo

grep -l "Language: German" $filename
if [ $? -eq 0 ]; then
   mv $filename ../novels-de
	exit
fi

grep -l "Language: French" $filename
if [ $? -eq 0 ]; then
   mv $filename ../novels-fr
	exit
fi

grep -l "Language: Italian" $filename
if [ $? -eq 0 ]; then
   mv $filename ../novels-it
	exit
fi

grep -l "Language: Spanish" $filename
if [ $? -eq 0 ]; then
   mv $filename ../novels-es
	exit
fi

grep -l "Language: Chinese" $filename
if [ $? -eq 0 ]; then
   mv $filename ../novels-cn
	exit
fi

grep -l "Language: Latin" $filename
if [ $? -eq 0 ]; then
   mv $filename ../novels-la
	exit
fi

grep -l "Language: Dutch" $filename
if [ $? -eq 0 ]; then
   mv $filename ../novels-nl
	exit
fi

grep -l "Language: Swedish" $filename
if [ $? -eq 0 ]; then
   mv $filename ../novels-sv
	exit
fi

grep -l "Language: Portuguese" $filename
if [ $? -eq 0 ]; then
   mv $filename ../novels-pt
	exit
fi

grep -l "Language: English" $filename
if [ $? -ne 0 ]; then
   echo Suspected non-English, bailing
	exit
fi

./chapters.sh foo $base-

# mv foo $filename

mv $filename whole-books
mv $base-* split-books
rm foo
