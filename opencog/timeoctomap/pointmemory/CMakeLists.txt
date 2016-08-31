#
# define the point_memory library.
ADD_LIBRARY(point_memory SHARED
	PointMemorySCM.cc
)

TARGET_LINK_LIBRARIES(point_memory
	time_octomap
	spacetime
	spacetime-types
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS point_memory
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	PointMemorySCM.h
	DESTINATION "include/${PROJECT_NAME}/timeoctomap"
)

INSTALL (FILES
	pointmem.scm
	DESTINATION "share/${PROJECT_NAME}/scm/opencog/ato"
)

#ADD_GUILE_MODULE(point-memory.scm)
