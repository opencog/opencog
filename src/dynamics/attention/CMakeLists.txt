ADD_LIBRARY(attention STATIC
	ImportanceUpdatingAgent.cc
	ImportanceSpreadingAgent.cc
	HebbianLearningAgent.cc
	ForgettingAgent.cc
    STIDecayingAgent.cc
)

INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/util
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/query
)

TARGET_LINK_LIBRARIES(attention server)
