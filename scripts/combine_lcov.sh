#!/bin/bash
# This script is used to merge separate lcov output from each test.
# Once they are in a single file, we create the html output with genhtml
BASEDIR=.
rm -r ${BASEDIR}/lcov

ifiles=""
for f in ${BASEDIR}/coverage/*.info; do
    ifiles="$ifiles -a $f"
done
echo "Combining all lcov .info files into single file"
lcov --directory ${BASEDIR} $ifiles --output-file ${BASEDIR}/coverage/alltemp.info
echo "Removing coverage info for non-OpenCog files"
# Get the name of the current dir
BIN_DIR=${PWD##*/}
lcov --directory ${BASEDIR} --output-file ${BASEDIR}/coverage/all.info \
    --remove ${BASEDIR}/coverage/alltemp.info \
        /usr/include/\* \
        ${BIN_DIR}/tests/\*
rm ${BASEDIR}/coverage/alltemp.info

echo "Creating lcov html summary"
genhtml -s -o ${BASEDIR}/lcov --demangle-cpp --num-spaces 4 --title "OpenCog Coverage Analysis" ${BASEDIR}/coverage/all.info
echo "Moving coverage files to ${BASEDIR}/coverage/old"
mkdir -p ${BASEDIR}/coverage/old
for f in ${BASEDIR}/coverage/*.info; do
    mv $f ${BASEDIR}/coverage/old
done
echo "All done!"

