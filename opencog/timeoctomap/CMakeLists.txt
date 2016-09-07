ADD_LIBRARY (time_octomap SHARED
	AtomOcTree.cc
	AtomOcTreeNode.cc
	TimeOctomap.cc
)


TARGET_LINK_LIBRARIES(time_octomap
	${OCTOMAP_LIBRARY}
	${OCTOMAP_OCTOMATH_LIBRARY}
	${ATOMSPACE_LIBRARIES})

INSTALL (TARGETS time_octomap
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	AtomOcTree.h
	AtomOcTreeNode.h
	TimeOctomap.h
	DESTINATION "include/${PROJECT_NAME}/timeoctomap"
)

ADD_SUBDIRECTORY (pointmemory)
