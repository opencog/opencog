IF (HAVE_GUILE AND HAVE_LINK_GRAMMAR)
	ADD_SUBDIRECTORY (sureal)

	# microplanning depends on sureal, so should test after it
	ADD_SUBDIRECTORY (microplanning)
	ADD_SUBDIRECTORY (chatlang)
ENDIF (HAVE_GUILE AND HAVE_LINK_GRAMMAR)

IF (HAVE_VITERBI)
	ADD_SUBDIRECTORY (viterbi)
ENDIF (HAVE_VITERBI)

IF (HAVE_NOSETESTS)

	ADD_TEST(AnaphoraTest ${NOSETESTS_EXECUTABLE} -vs
		${CMAKE_SOURCE_DIR}/tests/nlp/anaphora)
	SET_TESTS_PROPERTIES(AnaphoraTest PROPERTIES ENVIRONMENT
		"PYTHONPATH=${PROJECT_SOURCE_DIR}/opencog/nlp/anaphora:${PROJECT_BINARY_DIR}/opencog/cython:/usr/local/share/opencog/python;GUILE_LOAD_PATH=${PROJECT_BINARY_DIR}/opencog:${PROJECT_BINARY_DIR}/opencog/scm:/usr/local/share/opencog/scm"
	)

ENDIF (HAVE_NOSETESTS)

IF (HAVE_STACK AND BUILD_LOJBAN)
	# Lojban is currently failing to build; see for example
	# http://61.92.69.39:8080/job/ci-opencog-master/518/console
	ADD_SUBDIRECTORY(lojban)
ENDIF (HAVE_STACK AND BUILD_LOJBAN)
