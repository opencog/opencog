

ADD_LIBRARY (space_server SHARED
	TimeServer.cc
	SpaceServer.cc
)

TARGET_LINK_LIBRARIES(space_server
	SpaceMap
)

INSTALL (FILES
	SpaceServer.h
	TimeServer.h
	DESTINATION "include/${PROJECT_NAME}/spatial/space_server"
)

