find_library(COGUTIL_LIBRARY
	NAMES cogutil
	PATH /usr/lib /usr/lib64 /usr/local/lib /usr/local/lib64)

INCLUDE (CheckIncludeFiles)

# The location of iberty.h varies according to the distro.
# Fine the right location.
find_path(
    COGUTIL_INCLUDE_DIR backtrace-symbols.h
    PATHS
        /usr/include
        /usr/local/include
        /usr/include/opencog/util
        /usr/local/include/opencog/util
)

set(CMAKE_REQUIRED_INCLUDES ${COGUTIL_INCLUDE_DIR})
CHECK_INCLUDE_FILES (backtrace-symbols.h HAVE_COGUTIL_H)

if (COGUTIL_LIBRARY AND HAVE_COGUTIL_H)
	set( COGUTIL_FOUND TRUE )
endif (COGUTIL_LIBRARY AND HAVE_COGUTIL_H)

if ( COGUTIL_FOUND )
	message( STATUS "Found CogUtil: ${COGUTIL_LIBRARY}")
else ( COGUTIL_FOUND )
	message( STATUS "CogUtil not found")
endif ( COGUTIL_FOUND )
