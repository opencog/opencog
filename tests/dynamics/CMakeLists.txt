INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/dynamics/attention
	${OPENCOG_SOURCE_DIR}/src/util
)

LINK_DIRECTORIES(
	${OPENCOG_BINARY_DIR}/src/atomspace
	${OPENCOG_BINARY_DIR}/src/server
	${OPENCOG_BINARY_DIR}/src/dynamics/attention
	${OPENCOG_BINARY_DIR}/src/util
)

LINK_LIBRARIES(util atomspace server attention)

ADD_CXXTEST(ImportanceUpdatingAgentUTest)
ADD_CXXTEST(STIDecayingAgentUTest)
