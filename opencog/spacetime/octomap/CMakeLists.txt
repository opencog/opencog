ADD_LIBRARY (atom_octomap SHARED
	AtomOcTree.cc
	AtomOcTreeNode.cc
	TimeOctomap.cc
)

TARGET_LINK_LIBRARIES(atom_octomap
	${OCTOMAP_LIBRARY}
	${OCTOMAP_OCTOMATH_LIBRARY}
)

INSTALL (TARGETS atom_octomap
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	AtomOcTree.h
	AtomOcTreeNode.h
	TimeOctomap.h
	DESTINATION "include/${PROJECT_NAME}/spacetime/octomap"
)
