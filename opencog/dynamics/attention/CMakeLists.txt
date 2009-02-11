# To make Boost ublas much more efficient.
ADD_DEFINITIONS(-DNDEBUG)

ADD_LIBRARY(attention SHARED
	AttentionModule
	ForgettingAgent
	HebbianUpdatingAgent
	ImportanceSpreadingAgent
	ImportanceDiffusionAgent
	ImportanceUpdatingAgent
	STIDecayingAgent
)

INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/util
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/query
)

TARGET_LINK_LIBRARIES(attention server)

INSTALL (TARGETS attention
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	AttentionModule.h
	ForgettingAgent.h
	HebbianUpdatingAgent.h
	ImportanceSpreadingAgent.h
	ImportanceDiffusionAgent.h
	ImportanceUpdatingAgent.h
	STIDecayingAgent.h
	DESTINATION "include/opencog/dynamics/attention"
)
