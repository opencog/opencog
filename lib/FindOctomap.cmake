set(OCTOMAP_LIBPATH
   /opt/ros/indigo/lib)

find_library(OCTOMAP_LIBRARY NAMES octomap
   PATH ${OCTOMAP_LIBPATH})

find_library(OCTOMAP_OCTOMATH_LIBRARY NAMES octomath
   PATH ${OCTOMAP_LIBPATH})

set(OCTOMAP_LIBRARIES
  ${OCTOMAP_LIBRARY}
  ${OCTOMAP_OCTOMATH_LIBRARY}
)

INCLUDE (CheckIncludeFiles)

find_path(OCTOMAP_INCLUDE_DIR octomap/octomap.h
	PATH /usr/include /opt/ros/indigo/include /usr/local/include 
)

message("heyheyhey ${OCTOMAP_INCLUDE_DIR}")

if (OCTOMAP_LIBRARY AND OCTOMAP_INCLUDE_DIR)
	set(OCTOMAP_FOUND TRUE)
endif (OCTOMAP_LIBRARY AND OCTOMAP_INCLUDE_DIR)

if ( OCTOMAP_FOUND )
	message(STATUS "Found Octomap at ${OCTOMAP_LIBRARY}")
else ( OCTOMAP_FOUND )
	message(STATUS "Octomap not found wooooooo")
endif ( OCTOMAP_FOUND )


