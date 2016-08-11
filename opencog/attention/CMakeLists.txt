# To make Boost ublas much more efficient.
ADD_DEFINITIONS(-DNDEBUG)

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
ADD_DEPENDENCIES(attention-types opencog_atom_types)
ADD_DEPENDENCIES(attention-types attention_atom_types)

TARGET_LINK_LIBRARIES(attention-types
	${ATOMSPACE_atombase_LIBRARY}
)

# --------------------------------------------------

# AttentionModule
ADD_LIBRARY(attention SHARED
    AttentionModule
    ImportanceDiffusionBase
    AFImportanceDiffusionAgent
    WAImportanceDiffusionAgent
    RentCollectionBaseAgent
    AFRentCollectionAgent
    WARentCollectionAgent

    ForgettingAgent
    MinMaxSTIUpdatingAgent
    FocusBoundaryUpdatingAgent
    HebbianCreationAgent
    HebbianUpdatingAgent

    #scm/StimulationAgent
)

ADD_DEPENDENCIES(attention attention-types)

TARGET_LINK_LIBRARIES(attention server ${ATOMSPACE_LIBRARIES})

INSTALL (TARGETS attention
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/attention_types.scm
	DESTINATION "${DATADIR}/scm/attention"
)

INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/atom_types.h
	AttentionModule.h
	ForgettingAgent.h
	HebbianCreationModule.h
	HebbianUpdatingAgent.h
	ImportanceSpreadingAgent.h
	ImportanceDiffusionAgent.h
	SpreadDecider.h
	SimpleImportanceDiffusionAgent.h
	ImportanceUpdatingAgent.h
	DESTINATION "include/${PROJECT_NAME}/attention"
)

INSTALL (TARGETS attention-types
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog/modules")
