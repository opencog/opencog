#!/bin/bash
# This file is a wrapper for running cxxtests and creating per test coverage
# results. It takes a long time, and tests added outside of the patched
# ADD_CXXTEST won't be included.
bindir=$1 # the root bin dir
name=$3 # name of the test (and executable)
dir=$2 # dir the the test is built in
echo "test $name" 1>&2
cd $bindir
mkdir -p coverage
rm coverage/$name.info

# is run in the BUILDDIR/tests directory so .. is the root
lcov --directory . --zerocounter
# run test
$dir/$name
ret_val=$?
# capture coverage data to info file
lcov --directory . --capture --output-file coverage/$name.info --test-name $name

exit $ret_val
