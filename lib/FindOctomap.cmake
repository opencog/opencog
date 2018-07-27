set(OCTOMAP_LIBPATH
	/opt/ros/kinetic/lib
	/usr/lib
	/usr/local/lib)

find_library(OCTOMAP_LIBRARY octomap PATH ${OCTOMAP_LIBPATH})
message(STATUS "Find octomap lib ${OCTOMAP_LIBRARY}")

find_library(OCTOMAP_OCTOMATH_LIBRARY octomath PATH ${OCTOMAP_LIBPATH})
message(STATUS "Find octomath lib ${OCTOMAP_OCTOMATH_LIBRARY}")

set(OCTOMAP_LIBRARIES
	${OCTOMAP_LIBRARY}
	${OCTOMAP_OCTOMATH_LIBRARY}
)

INCLUDE (CheckIncludeFiles)

find_path(OCTOMAP_INCLUDE_DIR octomap/octomap.h
	PATH /usr/include /opt/ros/kinetic/include
	/usr/local/include
)

if (OCTOMAP_LIBRARY AND OCTOMAP_INCLUDE_DIR)
	set(OCTOMAP_FOUND TRUE)
endif (OCTOMAP_LIBRARY AND OCTOMAP_INCLUDE_DIR)

if (OCTOMAP_FOUND)
	message(STATUS "Found octomap headers at ${OCTOMAP_INCLUDE_DIR}")
else (OCTOMAP_FOUND)
	message(STATUS "Octomap not found")
endif (OCTOMAP_FOUND)
