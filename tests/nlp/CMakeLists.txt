IF (HAVE_BANK)
	ADD_SUBDIRECTORY (fuzzy)
ENDIF (HAVE_BANK)

# Disable sureal and microplanning - Jan 2020
# These two modules have not been used in many years, and are unmaintained.
# Recent changes to the pattern matcher have exposed bugs in
# nlp/sureal/SuRealPMCB.cc that are not readily apparent.  I don't want
# to maintain the code that no one else wants to maintain, either, so
# I'm just going to stub this out, and let future generations find and
# fix the bug(s). My general guess is that sureal probably should NOT
# be using a customized pattern matcher, anyway; just use what's
# provided, instead of inventing something oddly different.
#IF (HAVE_URE AND HAVE_NLP)
#	ADD_SUBDIRECTORY (sureal)
#	# microplanning depends on sureal, so should test after it
#	ADD_SUBDIRECTORY (microplanning)
#ENDIF (HAVE_URE AND HAVE_NLP)

# Disable AnaphoraUTest (March 2023) It has not been used in more than
# a decade. It has dependencies on MindAgents, which were eliminated
# years ago.
#IF (HAVE_NOSETESTS)
#
#	# AnaphoraUTest fails on guile-2.2 with a bdw-gc error:
#	# "Too many root sets".  It appears to work for guile-3.0
#	IF (GUILE_FOUND AND (GUILE_VERSION_MAJOR EQUAL 3)
#			OR ((GUILE_VERSION_MAJOR EQUAL 2) AND
#				(GUILE_VERSION_MINOR GREATER 4)))
#		ADD_TEST(AnaphoraTest ${NOSETESTS_EXECUTABLE} -vs
#			${CMAKE_SOURCE_DIR}/tests/nlp/anaphora)
#		SET_TESTS_PROPERTIES(AnaphoraTest PROPERTIES ENVIRONMENT
#			"PYTHONPATH=${PYTHON_ROOT}:${PROJECT_SOURCE_DIR}/opencog/nlp/anaphora:${PROJECT_BINARY_DIR}/opencog/cython"
#		)
#	ENDIF ()
#
#ENDIF (HAVE_NOSETESTS)

IF (HAVE_STACK AND BUILD_LOJBAN)
	# Lojban is currently failing to build; see for example
	# http://61.92.69.39:8080/job/ci-opencog-master/518/console
	ADD_SUBDIRECTORY(lojban)
ENDIF (HAVE_STACK AND BUILD_LOJBAN)
