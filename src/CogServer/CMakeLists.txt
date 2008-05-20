SET(CMAKE_BUILD_TYPE Debug)

ADD_LIBRARY (CogServer STATIC
	CogServer.cc 
	SimpleNetworkServer.cc 
	ServerSocket.cc 
	CommandRequest.cc 
	CommandRequestProcessor.cc 
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/Util
	${OPENCOG_SOURCE_DIR}/src/AtomSpace
	${OPENCOG_SOURCE_DIR}/src/NMXml
	${OPENCOG_SOURCE_DIR}/src/CogServer
	${OPENCOG_SOURCE_DIR}/src/nlp/wsd
	${OPENCOG_SOURCE_DIR}/src/query
	${OPENCOG_SOURCE_DIR}/src/persist
	${CSOCKETS_INCLUDE_DIRS}
	${OPENSSL_INCLUDE_DIR}
	${IODBC_INCLUDE_DIRS}
)

ADD_EXECUTABLE(cogServer CogServerMain.cc)

TARGET_LINK_LIBRARIES(CogServer
	AtomSpace
	NMXml
	Util
	query
    wsd
	${IODBC_LIBRARIES}
	${CSOCKETS_LIBRARIES}
	${OPENSSL_LIBRARIES}
	${EXPAT_LIBRARIES}
)

TARGET_LINK_LIBRARIES(cogServer CogServer)
