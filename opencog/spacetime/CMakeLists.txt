

ADD_LIBRARY (space_server SHARED
	HandleToTemporalEntryMap.cc
	HandleTemporalPairEntry.cc
	HandleTemporalPair.cc
	SpaceServer.cc
	Temporal.cc
	TemporalEntry.cc
	TemporalMap.cc
	TemporalTable.cc
	TemporalToHandleSetMap.cc
	TimeServer.cc
)

TARGET_LINK_LIBRARIES(space_server SpaceMap atomspace)

INSTALL (FILES
	HandleToTemporalEntryMap.h
	HandleTemporalPairEntry.h
	HandleTemporalPair.h
	SpaceServer.h
	SpaceServerContainer.h
	Temporal.h
	TemporalEntry.h
	TemporalMap.h
	TemporalTable.h
	TemporalToHandleSetMap.h
	TimeServer.h
	DESTINATION "include/${PROJECT_NAME}/spacetime"
)

