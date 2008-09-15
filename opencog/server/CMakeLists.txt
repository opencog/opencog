INCLUDE_DIRECTORIES (
	${CSOCKETS_INCLUDE_DIRS}
	${ODBC_INCLUDE_DIRS}
)

ADD_LIBRARY (server SHARED
    BaseServer.cc
	CogServer.cc 
	SimpleNetworkServer.cc 
	ServerSocket.cc 
	CommandRequest.cc 
	CommandRequestProcessor.cc 
)

TARGET_LINK_LIBRARIES(server
	atomspace
	xml
	query
	schemes
	wsd
	persist
	${ODBC_LIBRARIES}
	${CSOCKETS_LIBRARIES}
	${OPENSSL_LIBRARIES}
	${EXPAT_LIBRARIES}
	${GUILE_LIBRARIES}
	${LIBMEMCACHED_LIBRARIES}
)

ADD_EXECUTABLE(cogserver CogServerMain.cc)

TARGET_LINK_LIBRARIES(cogserver server)
IF (WIN32)
	SET_TARGET_PROPERTIES(cogserver PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libctm")
ENDIF (WIN32)

INSTALL (TARGETS server cogserver
	RUNTIME DESTINATION "bin"
	LIBRARY DESTINATION "lib/opencog"
)

INSTALL (FILES
    BaseServer.h
	CallBackInterface.h
	CogServer.h
	CogServerRequest.h
	CommandRequest.h
	CommandRequestProcessor.h
	MindAgent.h
	NetworkServer.h
	RequestProcessor.h
	ServerSocket.h
	SimpleNetworkServer.h
	DESTINATION "include/opencog/server"
)
