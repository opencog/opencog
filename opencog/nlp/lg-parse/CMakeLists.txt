
INCLUDE_DIRECTORIES (
	${LINK_GRAMMAR_INCLUDE_DIRS}	# for LinkGrammar dictionary
	${UUID_INCLUDE_DIRS}
	${CMAKE_BINARY_DIR}           # for the NLP atom types
)

ADD_LIBRARY (lg-parse SHARED
	LGParseLink
)

ADD_DEPENDENCIES (lg-parse nlp_atom_types)

TARGET_LINK_LIBRARIES (lg-parse
	lg-dict-entry
	nlp-types
	${ATOMSPACE_smob_LIBRARY}
	${LINK_GRAMMAR_LIBRARY}
	${UUID_LIBRARIES}
)

INSTALL (TARGETS lg-parse DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_GUILE_MODULE (FILES
	lg-parse.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/nlp/"
)

ADD_GUILE_EXTENSION(SCM_CONFIG lg-parse "opencog-ext-path-lg-parse")