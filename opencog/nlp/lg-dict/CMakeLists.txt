
INCLUDE_DIRECTORIES (
	${LINK_GRAMMAR_INCLUDE_DIRS}	# for LinkGrammar dictionary
	${CMAKE_BINARY_DIR}           # for the NLP atom types
)

ADD_LIBRARY (lg-dict-entry SHARED
	LGDictExpContainer
	LGDictReader
	LGDictUtils
	LGDictNode
	LGDictEntry
)

ADD_LIBRARY (lg-dict SHARED
	LGDictSCM
)

ADD_DEPENDENCIES (lg-dict-entry nlp_atom_types)

TARGET_LINK_LIBRARIES (lg-dict-entry
	nlp-types
	${ATOMSPACE_smob_LIBRARY}
	${LINK_GRAMMAR_LIBRARY}
)

TARGET_LINK_LIBRARIES (lg-dict
	lg-dict-entry
	${ATOMSPACE_smob_LIBRARY}
)

INSTALL (TARGETS lg-dict-entry DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
INSTALL (TARGETS lg-dict DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_GUILE_MODULE (FILES
	lg-dict.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/nlp/lg-dict"
)

INSTALL (FILES
	LGDictNode.h
	LGDictEntry.h
	DESTINATION "include/${PROJECT_NAME}/nlp/lg-dict"
)
