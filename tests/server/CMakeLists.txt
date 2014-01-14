INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/opencog/util
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/server
	${EXPAT_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/server
)

LINK_LIBRARIES(#CogServerUTest
	server
)

ADD_CXXTEST(CogServerUTest)
ADD_CXXTEST(AgentUTest)
