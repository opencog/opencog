SET(CMAKE_BUILD_TYPE Debug)

FIND_PACKAGE(EXPAT REQUIRED)
FIND_PACKAGE(CSockets REQUIRED)

ADD_LIBRARY (CogServer STATIC
	CogServerSetup.cc 
	CogServer.cc 
	SimpleNetworkServer.cc 
	ServerSocket.cc 
	CommandRequest.cc 
	CommandRequestProcessor.cc 
	ImportanceUpdatingAgent.cc
	ImportanceSpreadingAgent.cc
	HebbianLearningAgent.cc
	ForgettingAgent.cc
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/Util
	${OPENCOG_SOURCE_DIR}/src/AtomSpace
	${OPENCOG_SOURCE_DIR}/src/NMXml
	${OPENCOG_SOURCE_DIR}/src/CogServer
	${OPENCOG_SOURCE_DIR}/src/query
	${CSOCKETS_INCLUDE_DIRS}
)

ADD_EXECUTABLE(cogServer CogServerMain.cc)

TARGET_LINK_LIBRARIES(CogServer AtomSpace query NMXml Util ${CSOCKETS_LIBRARIES} ${EXPAT_LIBRARIES} pthread ssl crypto dl)

TARGET_LINK_LIBRARIES(cogServer CogServer)
