#!/bin/bash
# This script is used to merge separate lcov output from each test.
# Once they are in a single file, we create the html output with genhtml
BASEDIR=.
rm -r ${BASEDIR}/lcov

ifiles=""
for f in ${BASEDIR}/coverage/*.info; do
    ifiles="$ifiles -a $f"
done
lcov --directory ${BASEDIR} $ifiles --output-file ${BASEDIR}/coverage/alltemp.info
lcov --directory ${BASEDIR} --remove ${BASEDIR}/coverage/alltemp.info /usr/include/\* --output-file ${BASEDIR}/coverage/all.info
rm ${BASEDIR}/coverage/alltemp.info

genhtml -s -o ${BASEDIR}/lcov --demangle-cpp --num-spaces 4 --title "OpenCog Coverage Analysis" ${BASEDIR}/coverage/all.info
mkdir -p ${BASEDIR}/coverage/old
for f in ${BASEDIR}/coverage/*.info; do
    mv $f ${BASEDIR}/coverage/old
done

