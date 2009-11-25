INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/dynamics/attention
	${OPENCOG_SOURCE_DIR}/src/util
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/src/atomspace
	${OPENCOG_BINARY_DIR}/src/server
	${OPENCOG_BINARY_DIR}/src/dynamics/attention
	${OPENCOG_BINARY_DIR}/src/util
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
