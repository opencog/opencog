ADD_LIBRARY(attention STATIC
	ImportanceUpdatingAgent.cc
	ImportanceSpreadingAgent.cc
	HebbianLearningAgent.cc
	ForgettingAgent.cc
    STIDecayingAgent.cc
)

INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/Util
	${OPENCOG_SOURCE_DIR}/src/AtomSpace
	${OPENCOG_SOURCE_DIR}/src/CogServer
	${OPENCOG_SOURCE_DIR}/src/query
)

TARGET_LINK_LIBRARIES(attention CogServer)
