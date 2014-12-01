#!/bin/sh
#
# Simple script to run the cogserver with the default configuration
#
# It assumes by default that the building directory name is 'build`
# otherwise the first command argument can be given to overwrite it

branch_dir=../
build_dir_name=build
if [ $# -ge 1 ]    # one or more command line arguments
then
    # only the first argument is considered the others are ignored
    build_dir_name=$1
fi
build_dir=${branch_dir}/${build_dir_name}

${build_dir}/opencog/server/cogserver -c ../lib/opencog.conf
