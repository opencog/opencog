#!/bin/bash

# Takes a README file and converts into an appropriate
# doxygen comment block. Like:
# //---
# /*! \page directory_path_with_underscores last_dir
# 
# From directory \ref dir_ref.
# 
# Sub-modules:
# - \subpage subdir1
# - \subpage subdir2
#
# <include README contents>
#
# */
# //----

SOURCE=$1
SOURCE_PATH=`dirname $1`
SOURCE_NAME=`basename $1`
DEPTH=`echo $SOURCE_PATH | awk -F "/" '{ print NF }'`
# e.g. atomspace
LAST_DIR=`echo "$SOURCE_PATH" | awk -F "/" '{ print $NF }'`

# find root dir (from relation to script dir)
# e.g. /home/user/src/opencog
ROOT_SRC_DIR=`dirname $0 | awk -F "/" '{ for (i=2; i<NF; i++) printf "/"$i; \
print "" }'`

# doxygen passes the full path, so find root...
# found via relation to script path... not sure if there is a
# better way?
if [ "${SOURCE_PATH}" = "${ROOT_SRC_DIR}" ]; then
    IS_MAINPAGE=1
fi

if [ -n "${IS_MAINPAGE}" ]; then
    echo "//---- 
/*! @mainpage
"
    
    tail -n+4 $SOURCE 
# need to figure out way of adding reference labels to section command
    # | sed -n '1h;1!H;${;g;s/\([^\n]\+\)\n-\+/@section \1/g;p;}'
    
    cat $SOURCE_PATH/AUTHORS

    echo "
@section download Download
http://opencog.org/wiki/BuildingOpenCog

"

else
# e.g. opencog/atomspace
    IN_CODE_DIR="${SOURCE_PATH#$ROOT_SRC_DIR/}"
#    echo $IN_CODE_DIR
# e.g. opencog_atomspace
    PATH_UNDERSCORE=`echo $IN_CODE_DIR | sed -e 's/\//_/g'`
#    echo $PATH_UNDERSCORE
    echo "//---- 
/*! @page ${PATH_UNDERSCORE} ${LAST_DIR}
From directory \\ref ${SOURCE_PATH#$ROOT_SRC_DIR/}/
    "
fi
echo "@section submodules Components:"

# Now we have to find all sub dirs that contain README
for i in `find ${SOURCE_PATH}/ -name README | sort`
do
    SUBDIR_PATH=`dirname $i`
    # crappy way to check exclusions 
    # don't include the current README
    #if [ `dirname $i` != "./README"  ]; then
    # don't include the bzr dir (it has a README) 
    if [ `echo ${SUBDIR_PATH} | awk -F "/" '{ print $NF }'` != ".bzr"  ]; then
    # don't include doc dir (or maybe we should)
    #if [ `dirname $i | awk -F "/" '{print $2}'` != "doc" ]; then
        SUB_DEPTH=$(( ${DEPTH} + 1 ))
        if [ `echo ${SUBDIR_PATH} | awk -F "/" '{ print NF }'` -eq ${SUB_DEPTH} ]
        then
            SUBPATH_UNDERSCORE=`echo ${SUBDIR_PATH#$ROOT_SRC_DIR/} | sed -e 's/\//_/g'`
            # Remember to remove ._
            echo " - @subpage ${SUBPATH_UNDERSCORE#._}"
        fi
    fi; #fi; fi
done

if [ -z "${IS_MAINPAGE}" ]; then
    echo ""
    cat $SOURCE
fi

echo "*/
//----"

