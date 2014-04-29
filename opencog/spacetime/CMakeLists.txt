OPENCOG_ADD_ATOM_TYPES(atom_types.script
   atom_types.h
   atom_types.definitions
   atom_types.inheritance
   spacetime_types.scm)
ADD_CUSTOM_TARGET(spacetime_atom_types DEPENDS atom_types.h)

ADD_LIBRARY (spacetime SHARED
	atom_types_init.cc
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

ADD_DEPENDENCIES(spacetime opencog_atom_types)
ADD_DEPENDENCIES(spacetime spacetime_atom_types)

TARGET_LINK_LIBRARIES(spacetime SpaceMap server atomspace)

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

