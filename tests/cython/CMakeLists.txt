INCLUDE_DIRECTORIES(
	${Python3_INCLUDE_PATH}
)

IF (HAVE_NOSETESTS)

# Disable OpenPsi tests (Jan 2025) They are bit-rotted and don't run.
# They're probably been broken for a long time, but noticed only now.
#
#	IF (HAVE_OPENPSI)
#		ADD_TEST(OpenPsiCythonTest ${NOSETESTS_EXECUTABLE} -vs
#			${CMAKE_SOURCE_DIR}/tests/cython/openpsi/)
#		SET_PROPERTY(TEST OpenPsiCythonTest
#			PROPERTY ENVIRONMENT
#				"PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython:${PROJECT_BINARY_DIR}/opencog/openpsi:${PYTHON_ROOT}:${PYTHON_ROOT}/opencog"
#				"GUILE_LOAD_PATH=${GUILE_LOAD_PATH}"
#				)
#	ENDIF (HAVE_OPENPSI)

ENDIF (HAVE_NOSETESTS)
