INCLUDE_DIRECTORIES (
	${CMAKE_BINARY_DIR}       # for the NLP atom types
)

ADD_LIBRARY (sureal SHARED
	SuRealCache
	SuRealSCM
	SuRealPMCB
)

ADD_DEPENDENCIES (sureal
	nlp_atom_types            # also include the default types by dependencies
)

TARGET_LINK_LIBRARIES(sureal
	lg-dict
	nlp-types
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS sureal DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_GUILE_MODULE (sureal.scm
	surface-realization.scm
)
