INCLUDE_DIRECTORIES(
	${PYTHON_INCLUDE_PATH}
)

# The PythonModuleUTest is derived from the CogServer class, and so 
# cannot be tested unless the cogserver is actually being built!
IF (HAVE_SERVER)
	ADD_CXXTEST(PythonModuleUTest)
	ADD_DEPENDENCIES(PythonModuleUTest
		PythonModule
	)
	CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/cython/pymodule.conf
		${PROJECT_BINARY_DIR}/tests/cython/pymodule.conf)

	TARGET_LINK_LIBRARIES(PythonModuleUTest
		PythonModule
		server
		cogutil
		atomspace_cython
	)
	SET_TESTS_PROPERTIES(PythonModuleUTest
		PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")
ENDIF (HAVE_SERVER)


# The PyEvalUTest is derived from the CogServer class, and so 
# cannot be tested unless the cogserver is actually being built!
IF (HAVE_SERVER)
	ADD_CXXTEST(PyEvalUTest)
	CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/cython/pyeval.conf
		${PROJECT_BINARY_DIR}/tests/cython/pyeval.conf)

	TARGET_LINK_LIBRARIES(PyEvalUTest
		py-shell
		server
		cogutil
		atomspace_cython
	)
	SET_TESTS_PROPERTIES(PyEvalUTest
		PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")
ENDIF (HAVE_SERVER)


# The PythonEvalUTest tests PythonEval independent of the CogServer. It depends
# on the server being around for the Scheme code but it does not message
# through a "py-eval" server request.
#
ADD_CXXTEST(PythonEvalUTest)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/cython/pythoneval.conf
	${PROJECT_BINARY_DIR}/tests/cython/pythoneval.conf)

TARGET_LINK_LIBRARIES(PythonEvalUTest
	atomspace
	smob
	server
	PythonEval
	cogutil
	atomspace_cython
)
SET_TESTS_PROPERTIES(PythonEvalUTest
	PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")


# The PythonUtilitiesUTest tests C++ utility functions used from Cython bindings
# outside of those bindings so failures are easier to debug.
#
ADD_CXXTEST(PythonUtilitiesUTest)

TARGET_LINK_LIBRARIES(PythonUtilitiesUTest
	atomspace
	PythonEval
	cogutil
	utilities_cython
	atomspace_cython
)
SET_TESTS_PROPERTIES(PythonUtilitiesUTest
	PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")


IF (HAVE_NOSETESTS)

	# Basic test, just does the cython wrapper for opencog/util
	ADD_TEST(Cython ${NOSETESTS_EXECUTABLE} -vs
		${CMAKE_SOURCE_DIR}/tests/cython/)
	SET_TESTS_PROPERTIES(Cython
		PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")

	IF (HAVE_ATOMSPACE)
		ADD_TEST(CythonAtomSpace ${NOSETESTS_EXECUTABLE} -vs
			${CMAKE_SOURCE_DIR}/tests/cython/atomspace/)
		SET_TESTS_PROPERTIES(CythonAtomSpace
			PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")
	ENDIF (HAVE_ATOMSPACE)

	IF (HAVE_ATOMSPACE)
		ADD_TEST(CythonUtilities ${NOSETESTS_EXECUTABLE} -vs
			${CMAKE_SOURCE_DIR}/tests/cython/utilities/)
		SET_TESTS_PROPERTIES(CythonUtilities
			PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")
	ENDIF (HAVE_ATOMSPACE)

	IF (HAVE_ATOMSPACE)
		ADD_TEST(CythonBindlink ${NOSETESTS_EXECUTABLE} -vs
			${CMAKE_SOURCE_DIR}/tests/cython/bindlink/)
		SET_TESTS_PROPERTIES(CythonBindlink
			PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")
	ENDIF (HAVE_ATOMSPACE)

	IF (HAVE_ATOMSPACE AND HAVE_GUILE)
		ADD_TEST(CythonGuile ${NOSETESTS_EXECUTABLE} -vs
			${CMAKE_SOURCE_DIR}/tests/cython/guile/)
		SET_TESTS_PROPERTIES(CythonGuile
			PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")
	ENDIF (HAVE_ATOMSPACE AND HAVE_GUILE)

	IF (HAVE_SERVER)
		ADD_TEST(CythonServer ${NOSETESTS_EXECUTABLE} -vs
			${CMAKE_SOURCE_DIR}/tests/cython/server/)
		SET_TESTS_PROPERTIES(CythonServer
			PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython:${PROJECT_SOURCE_DIR}/tests/cython/agents")
	ENDIF(HAVE_SERVER)

	IF (HAVE_MOSES)
		ADD_TEST(CythonMoses ${NOSETESTS_EXECUTABLE} -vs
			${CMAKE_SOURCE_DIR}/tests/cython/moses/)
		SET_TESTS_PROPERTIES(CythonMoses
			PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")
	ENDIF (HAVE_MOSES)

ENDIF (HAVE_NOSETESTS)

