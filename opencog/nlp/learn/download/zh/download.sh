#!/bin/bash
#
# Download Chinese book texts; convert to utf8.
#
# The downloaded books are placed into the whole-books directory.
# Edit the `seq` line to change how many get downloaded.
# Edit the NUM line to change the range of books downloaded.
# After running the script, verify that the `tmp` directory has
# been deleted; if not, then whatever junk is in there needs to
# be dealt with.
#
mkdir tmp
mkdir whole-books

for f in `seq 1 12`; do
	let NUM=5000+$f

	# Download
	wget http://www.ixdzs.com/down/$NUM"_1"

	# use 7z to unzip; plain unzip garbles filenames
	7z x -otmp $NUM"_1"
	rm $NUM"_1"

	# Convert from GBK aka GB18030 to utf8
	# Need to convert the file contents and also the filename.
	find tmp/*.txt | while read f; \
		do \
			cat $f | iconv -f  GB18030 -t UTF-8 > foo; \
			mv foo `echo $f.utf8 | iconv -f  GB18030 -t UTF-8`; \
			rm $f; \
		done

	mv tmp/*.txt.utf8 whole-books
done

# Remove the tmp directory, if its empty.
rmdir tmp
