INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/src/atomspace
	${PROJECT_SOURCE_DIR}/src/server
	${PROJECT_SOURCE_DIR}/src/dynamics/attention
	${PROJECT_SOURCE_DIR}/src/util
)

LINK_DIRECTORIES (
	${PROJECT_BINARY_DIR}/src/atomspace
	${PROJECT_BINARY_DIR}/src/server
	${PROJECT_BINARY_DIR}/src/dynamics/attention
	${PROJECT_BINARY_DIR}/src/util
)

LINK_LIBRARIES (
	server
        attention
	${Boost_SIGNALS_LIBRARY}
	${GSL_LIBRARIES}
)

ADD_CXXTEST(ForgettingAgentUTest)
ADD_CXXTEST(HebbianUpdatingAgentUTest)
ADD_CXXTEST(ImportanceUpdatingAgentUTest)
IF (GSL_FOUND)
	ADD_CXXTEST(ImportanceDiffusionAgentUTest)
ENDIF (GSL_FOUND)
ADD_CXXTEST(STIDecayingAgentUTest)
