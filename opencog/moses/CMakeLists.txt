ADD_SUBDIRECTORY (moses)
ADD_SUBDIRECTORY (eda)
ADD_SUBDIRECTORY (main)

ADD_LIBRARY (MosesEda SHARED

	moses/build_knobs
	moses/knob_mapper
	moses/knobs
	moses/moses
	moses/representation
	moses/scoring
	moses/scoring_functions
	moses/types
	moses/using

    eda/field_set
    eda/initialization
    eda/local_structure
)

TARGET_LINK_LIBRARIES (MosesEda
	${COMBOREDUCT_LIBRARY}
	${LADSUTIL_LIBRARY}
)


INCLUDE_DIRECTORIES (
    ${CMAKE_SOURCE_DIR}/src
)

LINK_DIRECTORIES (
    ${CMAKE_BINARY_DIR}/src
)

#install library
INSTALL(TARGETS 
	${PROJECT_NAME}
	DESTINATION "lib/${PROJECT_NAME}" PERMISSIONS
	OWNER_READ OWNER_WRITE OWNER_EXECUTE
	GROUP_READ GROUP_EXECUTE
	WORLD_READ WORLD_EXECUTE
)

