
INCLUDE_DIRECTORIES (
	${LINK_GRAMMAR_INCLUDE_DIRS}	# for LinkGrammar dictionary
	${CMAKE_BINARY_DIR}           # for the NLP atom types
)

ADD_LIBRARY (lg-dict SHARED
	LGDictReader
)

ADD_DEPENDENCIES (lg-dict
	nlp_atom_types
)

TARGET_LINK_LIBRARIES (lg-dict smob ${LINK_GRAMMAR_LIBRARY})

ADD_LIBRARY (LGDictModule SHARED
	LGDictModule
)

TARGET_LINK_LIBRARIES (LGDictModule lg-dict server)

IF (WIN32)
   INSTALL (TARGETS lg-dict DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
   INSTALL (TARGETS lg-dict DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF ()
