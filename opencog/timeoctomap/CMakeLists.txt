IF (HAVE_OCTOMAP)
	ADD_LIBRARY (time_octomap SHARED
		AtomOcTree.cc
		AtomOcTreeNode.cc
		TimeOctomap.cc
	)


	TARGET_LINK_LIBRARIES(time_octomap
	${OCTOMAP_LIBRARY}
	${OCTOMAP_OCTOMATH_LIBRARY})
	
	TARGET_LINK_LIBRARIES(time_octomap
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

        ADD_LIBRARY(point_memory
                pointmemory/PointMemorySCM.cc
        )
        TARGET_LINK_LIBRARIES(point_memory
                time_octomap
                ${ATOMSPACE_LIBRARIES}
        )
        INSTALL (TARGETS point_memory
                 DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
        )
        INSTALL (FILES
                 pointmemory/PointMemorySCM.h
                 DESTINATION "include/${PROJECT_NAME}/timeoctomap"
        )

        ADD_GUILE_MODULE(pointmemory/point-memory.scm)
ENDIF (HAVE_OCTOMAP)
