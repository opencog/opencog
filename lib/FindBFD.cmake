find_library( DL_LIBRARY 	NAMES dl	PATH /usr/lib /usr/lib64 )
find_library( BFD_LIBRARY	NAMES bfd	PATH /usr/lib /usr/lib64 )

include(CheckCSourceCompiles)
check_c_source_compiles(
  "#include <bfd.h>
  int main(void) {
  return 0;
  }" BFD_WORKS)
if ( DL_LIBRARY AND BFD_LIBRARY AND BFD_WORKS)
	set( BFD_FOUND TRUE )
endif (DL_LIBRARY AND BFD_LIBRARY AND BFD_WORKS)

if ( BFD_FOUND )
	message( STATUS "Found libbfd: ${BFD_LIBRARY}")
endif ( BFD_FOUND )
