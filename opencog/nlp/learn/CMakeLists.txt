INCLUDE_DIRECTORIES (
	${CMAKE_BINARY_DIR}       # for the NLP atom types
)

ADD_LIBRARY (learn SHARED
	ObserveSentence
    ParseSentence
)

ADD_DEPENDENCIES (learn
	nlp_atom_types            # also include the default types by dependencies
)

TARGET_LINK_LIBRARIES(learn
	nlp-types
	${ICU_LIBRARY}
	${ATOMSPACE_LIBRARIES}
)

ADD_EXECUTABLE(parsetest
    ObserveSentence
    ParseSentence
)

TARGET_COMPILE_OPTIONS(parsetest PRIVATE -DTEST_MAIN -DDEBUG_MAIN)

TARGET_LINK_LIBRARIES(parsetest
    nlp-types
    ${ICU_LIBRARY}
    ${ATOMSPACE_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${GUILE_LIBRARIES}
)

INSTALL (TARGETS learn DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_SUBDIRECTORY (scm)
