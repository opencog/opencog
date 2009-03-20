CMAKE_POLICY(SET CMP0005 OLD)
ADD_DEFINITIONS(-D_REENTRANT -DTEST_CONFIG_FILE=\\"${CMAKE_CURRENT_SOURCE_DIR}/config.cfg\\")

SET(CMAKE_BUILD_TYPE Debug)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpace
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
        ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
	${OPENCOG_SOURCE_DIR}/opencog
	${OPENCOG_SOURCE_DIR}/include
	${OPENCOG_SOURCE_DIR}/include/filesystem
)

LINK_DIRECTORIES(
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Learning/LearningServerMessages
	${OPENCOG_BINARY_DIR}/opencog/util
)

ADD_CXXTEST(RouterMessageUTest)
TARGET_LINK_LIBRARIES(RouterMessageUTest MessagingSystem)

#ADD_CXXTEST(FileMessageCentralUTest)
#TARGET_LINK_LIBRARIES(FileMessageCentralUTest MessagingSystem)

#ADD_CXXTEST(StringMessageUTest)
#TARGET_LINK_LIBRARIES(StringMessageUTest MessagingSystem)

#ADD_CXXTEST(NetworkElementUTest)
#TARGET_LINK_LIBRARIES(NetworkElementUTest MessagingSystem)

#ADD_CXXTEST(ServerSocketUTest)
#TARGET_LINK_LIBRARIES(ServerSocketUTest MessagingSystem)

#ADD_CXXTEST(ClientSocketUTest)
#TARGET_LINK_LIBRARIES(ClientSocketUTest MessagingSystem)

#ADD_CXXTEST(RouterUTest)
#TARGET_LINK_LIBRARIES(RouterUTest MessagingSystem)

#ADD_CXXTEST(RouterServerSocketUTest)
#TARGET_LINK_LIBRARIES(RouterServerSocketUTest MessagingSystem)

#ADD_CXXTEST(RouterClientSocketUTest)
#TARGET_LINK_LIBRARIES(RouterClientSocketUTest MessagingSystem)

#ADD_CXXTEST(SpawnerUTest)
#TARGET_LINK_LIBRARIES(SpawnerUTest MessagingSystem)

#ADD_CXXTEST(DummyServerUTest)
#TARGET_LINK_LIBRARIES(DummyServerUTest MessagingSystem)
