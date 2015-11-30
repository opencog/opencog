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
	lg-dict
	nlp-types
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS sureal DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

INSTALL (FILES
	sureal.scm
	DESTINATION "${DATADIR}/scm/opencog/nlp")

INSTALL (FILES
	surface-realization.scm
	DESTINATION "${DATADIR}/scm/opencog/nlp/sureal")
