
INCLUDE_DIRECTORIES (
    ${LINK_GRAMMAR_INCLUDE_DIRS}    # for LinkGrammar dictionary
    ${CMAKE_BINARY_DIR}             # for the NLP atom types
)

ADD_LIBRARY (sureal SHARED
    LGDictReader
)

ADD_DEPENDENCIES (sureal
    nlp_atom_types
)

IF (HAVE_GUILE)
    TARGET_LINK_LIBRARIES (sureal smob ${LINK_GRAMMAR_LIBRARY})
ENDIF (HAVE_GUILE)


ADD_LIBRARY (LGDictModule SHARED
    LGDictModule
)
TARGET_LINK_LIBRARIES (LGDictModule sureal server)

IF (WIN32)
   INSTALL (TARGETS sureal DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
   INSTALL (TARGETS sureal DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF ()
