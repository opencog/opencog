

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

