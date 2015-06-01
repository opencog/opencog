INCLUDE_DIRECTORIES (
	${CMAKE_BINARY_DIR}       # for the NLP atom types
)

ADD_LIBRARY (sureal SHARED
	SuRealSCM
	SuRealPMCB
)

ADD_DEPENDENCIES (sureal
	nlp_atom_types            # also include the default types by dependencies
)

TARGET_LINK_LIBRARIES(sureal
	nlp-types
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS sureal DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
