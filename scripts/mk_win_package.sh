#!/bin/bash
#
# script to generate a Windows directory with all the tools and all
# necessary dlls to run OpenCog on a windows machine.
#
# For the moment only MOSES and feature-selection are included.
#
# Note that you must run that script after having compiled (supposedly
# under cygwin), it will not compile OpenCog for you.

if [ $# != 2 ]; then
    echo "Wrong number of arguments"
    echo "Usage: $0 BUILD_DIR PACKAGE_DIR"
    echo "where BUILD_DIR is the path where you have compiled OpenCog"
    echo "and PACKAGE_DIR is the name of the directory containing the tools and dll that you want to create"
    exit 1
fi

BUILD_DIR=$(readlink -f $1) # get the absolute path, no matter what
PACKAGE_DIR="$2"

# create package directory
mkdir "$PACKAGE_DIR"

# move all required cygwin dlls
CYGWIN_FILES=( cygboost_filesystem-mt-1_43.dll cygboost_system-mt-1_43.dll cyggomp-1.dll cygicuuc38.dll cygboost_program_options-mt-1_43.dll cygboost_thread-mt-1_43.dll cygicudata38.dll cygssp-0.dll cygwin1.dll cygboost_regex-mt-1_43.dll cyggcc_s-1.dll cygicui18n38.dll cygstdc++-6.dll )
for f in ${CYGWIN_FILES[@]}; do
    cp "/bin/$f" "$PACKAGE_DIR"
done

# move all OpenCog tools and dlls
OPENCOG_FILES=( opencog/util/cygutil.dll opencog/comboreduct/cygcomboreduct.dll opencog/learning/feature-selection/main/feature-selection.exe opencog/learning/moses/cygmoses.dll opencog/learning/moses/main/moses-exec.exe )
for f in ${OPENCOG_FILES[@]}; do
    cp "$BUILD_DIR/$f" "$PACKAGE_DIR"
done
