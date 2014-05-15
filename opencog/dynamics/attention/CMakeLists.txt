# To make Boost ublas much more efficient.
ADD_DEFINITIONS(-DNDEBUG)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/query
	${PROJECT_SOURCE_DIR}/opencog/server
	${PROJECT_SOURCE_DIR}/opencog/util
)

# --------------------------------------------------
OPENCOG_ADD_ATOM_TYPES(atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	attention_types.scm)
ADD_CUSTOM_TARGET(attention_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY(attention-types SHARED
	atom_types_init.cc
)

# Without this, parallel make will race and crap up the generated files.
ADD_DEPENDENCIES(attention-types opencog_atom_types)
ADD_DEPENDENCIES(attention-types attention_atom_types)

TARGET_LINK_LIBRARIES(attention-types atomspace)

# --------------------------------------------------
# AttentionModule
ADD_LIBRARY(attention SHARED
	AttentionModule
	ForgettingAgent
	HebbianUpdatingAgent
	ImportanceSpreadingAgent
	ImportanceDiffusionAgent
	ImportanceUpdatingAgent
)

TARGET_LINK_LIBRARIES(attention server)

# HebbianCreationModule
IF (TBB_FOUND)
    INCLUDE_DIRECTORIES (
	    ${TBB_INCLUDE_DIR}
    )

    LINK_DIRECTORIES(
	    ${TBB_INCLUDE_DIR}
	    ${TBB_LIBRARY_DIRS}
    )

    ADD_LIBRARY(hebbiancreation SHARED
	    HebbianCreationModule
    )

    TARGET_LINK_LIBRARIES(hebbiancreation
	    server
	    tbb
    )
ENDIF (TBB_FOUND)

IF (CYGWIN)
	INSTALL (TARGETS attention
		DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
	)
ELSE (CYGWIN)
	INSTALL (TARGETS attention
		LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
	)
ENDIF (CYGWIN)


INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/atom_types.h
	AttentionModule.h
	ForgettingAgent.h
	HebbianCreationModule.h
	HebbianUpdatingAgent.h
	ImportanceSpreadingAgent.h
	ImportanceDiffusionAgent.h
	ImportanceUpdatingAgent.h
	DESTINATION "include/${PROJECT_NAME}/dynamics/attention"
)
