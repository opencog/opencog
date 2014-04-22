find_library( IBERTY_LIBRARY	NAMES iberty	PATH /usr/lib /usr/lib64 )

INCLUDE (CheckIncludeFiles)

# The location of iberty.h varies according to the distro.
# Fine the right location.
find_path(
    IBERTY_INCLUDE_DIR libiberty.h
    PATHS
        /usr/include
        /usr/local/include
        /usr/include/libiberty
        /usr/local/include/libiberty
)

set(CMAKE_REQUIRED_INCLUDES ${IBERTY_INCLUDE_DIR})
CHECK_INCLUDE_FILES (libiberty.h HAVE_IBERTY_H)

if (IBERTY_LIBRARY AND HAVE_IBERTY_H)
	set( IBERTY_FOUND TRUE )
endif (IBERTY_LIBRARY AND HAVE_IBERTY_H)

if ( IBERTY_FOUND )
	message( STATUS "Found libiberty: ${IBERTY_LIBRARY}")
else ( IBERTY_FOUND )
	message( STATUS "IBERTY not found")
endif ( IBERTY_FOUND )
