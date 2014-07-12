OPENCOG_ADD_ATOM_TYPES(atom_types.script
   atom_types.h
   atom_types.definitions
   atom_types.inheritance
   spacetime_types.scm)
ADD_CUSTOM_TARGET(spacetime_atom_types DEPENDS atom_types.h)

ADD_LIBRARY (spacetime-types SHARED
	atom_types_init.cc
)

ADD_DEPENDENCIES(spacetime-types opencog_atom_types)
ADD_DEPENDENCIES(spacetime-types spacetime_atom_types)

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

TARGET_LINK_LIBRARIES(spacetime spacetime-types SpaceMap server atomspace)

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

