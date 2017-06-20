OPENCOG_ADD_ATOM_TYPES(atom_types.script
   atom_types.h
   atom_types.definitions
   atom_types.inheritance
   spacetime_types.scm
   spacetime_types.pyx)
ADD_CUSTOM_TARGET(spacetime_atom_types DEPENDS atom_types.h)

ADD_LIBRARY (spacetime-types SHARED
	atom_types_init.cc
)

TARGET_LINK_LIBRARIES(spacetime-types
	${ATOMSPACE_atombase_LIBRARY}
)

ADD_DEPENDENCIES(spacetime-types spacetime_atom_types)

INSTALL (TARGETS spacetime-types
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	spacetime.scm
	DESTINATION "${DATADIR}/scm/opencog"
)

INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/spacetime_types.scm
	DESTINATION "${DATADIR}/scm/opencog/spacetime"
)

# FIXME -- actually, the timeserver does not depend on octomap,
# and could be built just fine without it. However, its not clear
# why we need to fix this just right now.
IF (HAVE_OCTOMAP)
	ADD_LIBRARY (spacetime SHARED
		HandleToTemporalEntryMap.cc
		HandleTemporalPairEntry.cc
		HandleTemporalPair.cc
		SpaceServer.cc
		SpaceTime.cc
		Temporal.cc
		TemporalEntry.cc
		TemporalMap.cc
		TemporalTable.cc
		TemporalToHandleSetMap.cc
		TimeServer.cc
	)

	TARGET_LINK_LIBRARIES(spacetime
		spacetime-types
		SpaceMap
		server
		${ATOMSPACE_LIBRARIES})

	INSTALL (TARGETS spacetime
		DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
	)

	INSTALL (FILES
		HandleToTemporalEntryMap.h
		HandleTemporalPairEntry.h
		HandleTemporalPair.h
		SpaceServer.h
		SpaceTime.h
		SpaceServerContainer.h
		Temporal.h
		TemporalEntry.h
		TemporalMap.h
		TemporalTable.h
		TemporalToHandleSetMap.h
		TimeServer.h
		DESTINATION "include/${PROJECT_NAME}/spacetime"
	)
ENDIF (HAVE_OCTOMAP)
