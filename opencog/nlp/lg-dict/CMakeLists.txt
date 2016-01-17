
INCLUDE_DIRECTORIES (
	${LINK_GRAMMAR_INCLUDE_DIRS}	# for LinkGrammar dictionary
	${CMAKE_BINARY_DIR}           # for the NLP atom types
)

ADD_LIBRARY (lg-dict SHARED
	LGDictSCM
	LGDictReader
	LGDictUtils
	LGDictExpContainer
)

ADD_DEPENDENCIES (lg-dict
	nlp_atom_types
)

TARGET_LINK_LIBRARIES (lg-dict
	nlp-types
	${ATOMSPACE_smob_LIBRARY}
	${LINK_GRAMMAR_LIBRARY}
)

INSTALL (TARGETS lg-dict DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_GUILE_MODULE (lg-dict.scm)
