IF (HAVE_NLP)
	ADD_SUBDIRECTORY (fuzzy)
ENDIF (HAVE_NLP)

IF (HAVE_GUILE AND HAVE_LINK_GRAMMAR)
	ADD_SUBDIRECTORY (sureal)

	# microplanning depends on sureal, so should test after it
	ADD_SUBDIRECTORY (microplanning)
ENDIF (HAVE_GUILE AND HAVE_LINK_GRAMMAR)

IF (HAVE_VITERBI)
	ADD_SUBDIRECTORY (viterbi)
ENDIF (HAVE_VITERBI)

IF (HAVE_NOSETESTS)

	# AnaphoraUTest fails on guile-2.2 with a bdw-gc error:
	# "Too many root sets".  It appears to work for guile-3.0
	IF (GUILE_FOUND AND (GUILE_VERSION_MAJOR EQUAL 3)
			OR ((GUILE_VERSION_MAJOR EQUAL 2) AND
				(GUILE_VERSION_MINOR GREATER 4)))
		ADD_TEST(AnaphoraTest ${NOSETESTS_EXECUTABLE} -vs
			${CMAKE_SOURCE_DIR}/tests/nlp/anaphora)
		SET_TESTS_PROPERTIES(AnaphoraTest PROPERTIES ENVIRONMENT
			"PYTHONPATH=${PYTHON_ROOT}:${PROJECT_SOURCE_DIR}/opencog/nlp/anaphora:${PROJECT_BINARY_DIR}/opencog/cython"
		)
	ENDIF ()

ENDIF (HAVE_NOSETESTS)

IF (HAVE_STACK AND BUILD_LOJBAN)
	# Lojban is currently failing to build; see for example
	# http://61.92.69.39:8080/job/ci-opencog-master/518/console
	ADD_SUBDIRECTORY(lojban)
ENDIF (HAVE_STACK AND BUILD_LOJBAN)
