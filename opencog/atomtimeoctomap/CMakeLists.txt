IF (HAVE_OCTOMAP)
	ADD_LIBRARY (time_space_atom SHARED
		AtomOcTree.cc
		AtomOcTreeNode.cc
		TimeSpaceAtom.cc
	)


	TARGET_LINK_LIBRARIES(time_space_atom
	${OCTOMAP_LIBRARY}
	${OCTOMAP_OCTOMATH_LIBRARY})
	
	TARGET_LINK_LIBRARIES(time_space_atom
		${ATOMSPACE_LIBRARIES})

	INSTALL (TARGETS time_space_atom
		DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
	)

	INSTALL (FILES
		AtomOcTree.h
		AtomOcTreeNode.h
		TimeSpaceAtom.h
		DESTINATION "include/${PROJECT_NAME}/atomtimeoctomap"
	)

ENDIF (HAVE_OCTOMAP)
