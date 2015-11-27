set(OCTOMAP_LIBPATH
	/opt/ros/indigo/lib
	/usr/lib
	/usr/local/lib)

find_library(OCTOMAP_LIBRARY octomap
	PATH ${OCTOMAP_LIBPATH})
message("Find octomap lib ${OCTOMAP_LIBRARY}")

find_library(OCTOMAP_OCTOMATH_LIBRARY octomath
	PATH ${OCTOMAP_LIBPATH})
message("Find octomath lib ${OCTOMAP_OCTOMATH_LIBRARY}")


set(OCTOMAP_LIBRARIES
	${OCTOMAP_LIBRARY}
	${OCTOMAP_OCTOMATH_LIBRARY}
)

INCLUDE (CheckIncludeFiles)

find_path(OCTOMAP_INCLUDE_DIR octomap/octomap.h
	PATH /usr/include /opt/ros/indigo/include
	/usr/local/include
)

if (OCTOMAP_LIBRARY AND OCTOMAP_INCLUDE_DIR)
	set(OCTOMAP_FOUND TRUE)
endif (OCTOMAP_LIBRARY AND OCTOMAP_INCLUDE_DIR)

if ( OCTOMAP_FOUND )
	message(STATUS "Found Octomap at ${OCTOMAP_LIBRARY}")
else ( OCTOMAP_FOUND )
	message(STATUS "Octomap not found")
endif ( OCTOMAP_FOUND )
