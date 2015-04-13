# find_library( DL_LIBRARY 	NAMES dl	PATH /usr/lib /usr/lib64 )
find_library( BFD_LIBRARY	NAMES bfd	PATH /usr/lib /usr/lib64 )

# # Check to see if bfd cn compile and link ...
# include(CheckCSourceCompiles)
# check_c_source_compiles(
#   "#include <bfd.h>
#   int main(void) {
#   return 0;
#   }" BFD_WORKS)

#if (DL_LIBRARY AND BFD_LIBRARY AND BFD_WORKS)
#	set( BFD_FOUND TRUE )
#endif (DL_LIBRARY AND BFD_LIBRARY AND BFD_WORKS)

INCLUDE (CheckIncludeFiles)
CHECK_INCLUDE_FILES (bfd.h HAVE_BFD_H)

if (BFD_LIBRARY AND HAVE_BFD_H)
	set( BFD_FOUND TRUE )
endif (BFD_LIBRARY AND HAVE_BFD_H)

if ( BFD_FOUND )
	message( STATUS "Found libbfd: ${BFD_LIBRARY}")
else ( BFD_FOUND )
	message( STATUS "BFD not found")
endif ( BFD_FOUND )
