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
		PROPERTIES ENVIRONMENT
	"PYTHONPATH=/usr/local/share/opencog/python:${PROJECT_SOURCE_DIR}/opencog/nlp/anaphora:${PROJECT_SOURCE_DIR}/opencog/python:${PROJECT_BINARY_DIR}/opencog/cython")

ENDIF (HAVE_NOSETESTS)
