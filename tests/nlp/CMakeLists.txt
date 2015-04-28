IF (HAVE_GUILE AND HAVE_LINK_GRAMMAR)
	ADD_SUBDIRECTORY (sureal)

	# microplanning depends on sureal, so should test after it
	ADD_SUBDIRECTORY (microplanning)
ENDIF (HAVE_GUILE AND HAVE_LINK_GRAMMAR)

IF (HAVE_VITERBI)
	ADD_SUBDIRECTORY (viterbi)
ENDIF (HAVE_VITERBI)

IF (HAVE_NOSETESTS)

    # Basic test, just does the cython wrapper for opencog/util
    ADD_TEST(AnaphoraTest ${NOSETESTS_EXECUTABLE} -vs
        ${CMAKE_SOURCE_DIR}/tests/nlp/anaphora)
    SET_TESTS_PROPERTIES(AnaphoraTest
        PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython:${PROJECT_SOURCE_DIR}/opencog/nlp")

ENDIF (HAVE_NOSETESTS)

