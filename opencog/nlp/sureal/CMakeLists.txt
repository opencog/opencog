INCLUDE_DIRECTORIES (
	${CMAKE_BINARY_DIR}       # for the NLP atom types
)

ADD_LIBRARY (sureal SHARED
	SuRealCache.cc
	SuRealSCM.cc
	SuRealPMCB.cc
)

ADD_DEPENDENCIES (sureal
	nlp_oc_atom_types      # also include the default types by dependencies
)

TARGET_LINK_LIBRARIES(sureal
	lg-dict
	neighbors
	nlp-types
	nlp-oc-types
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS sureal DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_GUILE_MODULE (FILES
	sureal.scm
	surface-realization.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/nlp/sureal"
)

ADD_GUILE_EXTENSION(SCM_CONFIG sureal "opencog-ext-path-sureal")
