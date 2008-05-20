INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/AtomSpace
	${OPENCOG_SOURCE_DIR}/src/CogServer
	${OPENCOG_SOURCE_DIR}/src/dynamics/attention
	${OPENCOG_SOURCE_DIR}/src/Util
)

LINK_DIRECTORIES(
	${OPENCOG_BINARY_DIR}/src/AtomSpace
	${OPENCOG_BINARY_DIR}/src/CogServer
	${OPENCOG_BINARY_DIR}/src/dynamics/attention
	${OPENCOG_BINARY_DIR}/src/Util
)

LINK_LIBRARIES(Util AtomSpace CogServer attention)
IF(WIN32)
	LINK_LIBRARIES(winmm)
ENDIF(WIN32)

ADD_CXXTEST(ImportanceUpdatingAgentUTest)
ADD_CXXTEST(STIDecayingAgentUTest)
