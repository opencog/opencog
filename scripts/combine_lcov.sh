#!/bin/bash
# This script is used to merge separate lcov output from each test.
# Once they are in a single file, we create the html output with genhtml
rm -r ../lcov

ifiles=""
for f in ../coverage/*.info; do
    ifiles="$ifiles -a $f"
done
lcov --directory . $ifiles --output-file ../coverage/alltemp.info
lcov --directory . --remove ../coverage/alltemp.info /usr/include/\* --output-file ../coverage/all.info
rm ../coverage/alltemp.info

genhtml -s -o ../lcov --demangle-cpp --num-spaces 4 --title "OpenCog Coverage Analysis" ../coverage/all.info
mkdir -p ../coverage/old
for f in ../coverage/*.info; do
    mv $f ../coverage/old
done

