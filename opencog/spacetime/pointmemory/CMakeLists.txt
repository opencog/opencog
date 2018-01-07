#
# define the point_memory library.
ADD_LIBRARY(point_memory SHARED
	PointMemorySCM.cc
)

TARGET_LINK_LIBRARIES(point_memory
	atom_octomap
	spacetime-types
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS point_memory
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

ADD_GUILE_MODULE (FILES
	pointmem.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/pointmem"
)
