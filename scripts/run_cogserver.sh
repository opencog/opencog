#!/bin/bash
#
# Simple script to run the cogserver with the default configuration
#
# It assumes by default that the building directory name is 'build`
# otherwise the first command argument can be given to overwrite it

set -u
# set -x

prg_path="$(readlink -f "$0")"
prg_dir="$(dirname "$prg_path")"
branch_dir="$prg_dir/.."
if [[ $# == 0 ]]; then # No argument
    build_dir_name=build
elif [[ $# == 1 ]]; then # One argument
    build_dir_name="$1"
else # More than one command line arguments
    echo "Usage: $0 [BUILD_DIR_NAME=build]"
    exit 1
fi

build_dir="${branch_dir}/${build_dir_name}"

"${build_dir}/opencog/server/cogserver" \
    -c "${branch_dir}/lib/opencog.conf"
