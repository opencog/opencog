# To make Boost ublas much more efficient.
ADD_DEFINITIONS(-DNDEBUG)
#ADD_DEFINITIONS(-DLOG_AV_STAT)

# --------------------------------------------------
OPENCOG_ADD_ATOM_TYPES(atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	attention_types.scm
	attention_types.pyx)
ADD_CUSTOM_TARGET(attention_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY(attention-types SHARED
	atom_types_init.cc
)

# Without this, parallel make will race and crap up the generated files.
ADD_DEPENDENCIES(attention-types attention_atom_types)

TARGET_LINK_LIBRARIES(attention-types
	${ATOMSPACE_atombase_LIBRARY}
)

# --------------------------------------------------

# AttentionModule
ADD_LIBRARY(attention SHARED
	AttentionParamQuery
 	AttentionUtils
	AttentionModule
	ImportanceDiffusionBase
	AFImportanceDiffusionAgent
	WAImportanceDiffusionAgent
	RentCollectionBaseAgent
	AFRentCollectionAgent
	WARentCollectionAgent

	ForgettingAgent
	HebbianCreationAgent
	HebbianUpdatingAgent

	#scm/StimulationAgent
)

TARGET_LINK_LIBRARIES(attention attention-types server ${ATOMSPACE_LIBRARIES})


INSTALL (TARGETS attention-types
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (TARGETS attention
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	${CMAKE_CURRENT_SOURCE_DIR}/attention.scm
	DESTINATION "${DATADIR}/scm/opencog"
)

INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/attention_types.scm
	${CMAKE_CURRENT_SOURCE_DIR}/default-param-values.scm
	DESTINATION "${DATADIR}/scm/opencog/attention"
)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/default-param-values.scm
	${CMAKE_CURRENT_BINARY_DIR}/default-param-values.scm
)

INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/atom_types.h
	AttentionModule.h
	AttentionStat.h
	ForgettingAgent.h
	HebbianUpdatingAgent.h
	DESTINATION "include/${PROJECT_NAME}/attention"
)
