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
	${GSL_LIBRARIES}
    ${CSOCKETS_LIBRARIES}
    ${OPENSSL_LIBRARIES}
)

ADD_CXXTEST(ImportanceUpdatingAgentUTest)
ADD_CXXTEST(ForgettingAgentUTest)
ADD_CXXTEST(ImportanceDiffusionAgentUTest)
ADD_CXXTEST(HebbianLearningAgentUTest)
ADD_CXXTEST(STIDecayingAgentUTest)
