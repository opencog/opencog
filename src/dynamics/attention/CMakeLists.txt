SET(CMAKE_BUILD_TYPE Debug)

ADD_LIBRARY (attention STATIC
	ImportanceUpdatingAgent.cc
	ImportanceSpreadingAgent.cc
	HebbianLearningAgent.cc
	ForgettingAgent.cc
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/Util
	${OPENCOG_SOURCE_DIR}/src/AtomSpace
	${OPENCOG_SOURCE_DIR}/src/CogServer
	${OPENCOG_SOURCE_DIR}/src/query
)

TARGET_LINK_LIBRARIES(attention CogServer query AtomSpace Util)
#pthread ssl crypto dl)
