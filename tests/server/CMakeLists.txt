INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/opencog/util
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/server
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/server
)

LINK_LIBRARIES(#CogServerUTest
	server
)

# CogserverUTest hangs, now that time is computed correctly :-(
# ADD_CXXTEST(CogServerUTest)
ADD_CXXTEST(AgentUTest)
