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
	${ATOMSPACE_atomcore_LIBRARY}
	${ATOMSPACE_atombase_LIBRARY}
	${ATOMSPACE_atomproto_LIBRARY}
	${ATOMSPACE_atomtypes_LIBRARY}
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

TARGET_LINK_LIBRARIES(attention
	attention-types
	neighbors
	server
	${ATOMSPACE_LIBRARIES})


INSTALL (TARGETS attention-types
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (TARGETS attention
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

ADD_GUILE_MODULE (FILES
	attention.scm
	default-param-values.scm
	${CMAKE_CURRENT_BINARY_DIR}/attention_types.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/attention"
	DEPENDS attention_atom_types
)

INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/atom_types.h
	AttentionModule.h
	AttentionStat.h
	ForgettingAgent.h
	HebbianUpdatingAgent.h
	DESTINATION "include/${PROJECT_NAME}/attention"
)
