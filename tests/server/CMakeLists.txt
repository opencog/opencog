INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/opencog/util
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/cogserver/server
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/cogserver/server
)

LINK_LIBRARIES(
	server
	${ATOMSPACE_LIBRARIES}
	${Boost_SYSTEM_LIBRARY}
)

ADD_CXXTEST(CogServerUTest)
ADD_CXXTEST(ShellUTest)
ADD_CXXTEST(AgentUTest)
