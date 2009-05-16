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
PATH_UNDERSCORE=`echo $SOURCE_PATH | sed -e 's/\//_/g'`
LAST_DIR=`echo "$SOURCE_PATH" | awk -F "/" '{ print $NF }'`

# find root dir
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
    cat $SOURCE
    
    cat $SOURCE_PATH/AUTHORS

    echo "
@section download Download
http://opencog.org/wiki/BuildingOpenCog

"

else
    echo "//---- 
/*! @page ${PATH_UNDERSCORE} ${LAST_DIR}
From directory \\ref ./${SOURCE_PATH#$ROOT_SRC_DIR/}/
    "
fi
echo "@section submodules Components:"

# Now we have to find all sub dirs that contain README
for i in `find ${SOURCE_PATH}/ -name README | sort`
do
    # crappy way to check exclusions 
    # don't include the current README
    #if [ `dirname $i` != "./README"  ]; then
    # don't include the bzr dir (it has a README) 
    if [ `dirname $i | awk -F "/" '{ print $NF }'` != ".bzr"  ]; then
    # don't include doc dir (or maybe we should)
    #if [ `dirname $i | awk -F "/" '{print $2}'` != "doc" ]; then
        SUB_DEPTH=$(( ${DEPTH} + 1 ))
        if [ `dirname $i | awk -F "/" '{ print NF }'` -eq ${SUB_DEPTH} ]
        then
            SUBPATH_UNDERSCORE=`dirname $i | sed -e 's/\//_/g'`
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

