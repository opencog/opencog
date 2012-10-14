

# XXX FIXME TODO Until such time that the learning-server messages are
# removed from the spacemap, we have to build static, to avoid circular
# dependencies. That's just plain wrong.
# ADD_LIBRARY (space_server SHARED
ADD_LIBRARY (space_server STATIC
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

TARGET_LINK_LIBRARIES(space_server
	SpaceMap
)

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
	DESTINATION "include/${PROJECT_NAME}/spatial/space_server"
)

