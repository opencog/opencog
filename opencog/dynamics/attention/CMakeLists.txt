OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)
ADD_LIBRARY(attention SHARED
	atom_types
	AttentionModule
	ForgettingAgent
	HebbianLearningAgent
	ImportanceSpreadingAgent
	ImportanceDiffusionAgent.cc
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
	atom_types.h
	AttentionModule.h
	ForgettingAgent.h
	HebbianLearningAgent.h
	ImportanceSpreadingAgent.h
	ImportanceDiffusionAgent.h
	ImportanceUpdatingAgent.h
	STIDecayingAgent.h
	DESTINATION "include/opencog/dynamics/attention"
)
