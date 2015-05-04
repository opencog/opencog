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
		atomspace_cython
		${COGUTIL_LIBRARY}
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
		atomspace_cython
		${COGUTIL_LIBRARY}
	)
	SET_TESTS_PROPERTIES(PyEvalUTest
		PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")

	# The PythonEvalUTest tests PythonEval independent of the CogServer.
	# It depends on the server for some reason but it does not message
	# through a "py-eval" server request.
	#
	ADD_CXXTEST(PythonEvalUTest)
	CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/cython/pythoneval.conf
		${PROJECT_BINARY_DIR}/tests/cython/pythoneval.conf)

	TARGET_LINK_LIBRARIES(PythonEvalUTest
		PythonEval
		atomspace_cython
		server
		${ATOMSPACE_LIBRARIES}
		${COGUTIL_LIBRARY}
	)
	SET_TESTS_PROPERTIES(PythonEvalUTest
		PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")

ENDIF (HAVE_SERVER)

# The PythonUtilitiesUTest tests C++ utility functions used from Cython bindings
# outside of those bindings so failures are easier to debug.
#
ADD_CXXTEST(PythonUtilitiesUTest)

TARGET_LINK_LIBRARIES(PythonUtilitiesUTest
	PythonEval
	utilities_cython
	atomspace_cython
	${ATOMSPACE_LIBRARIES}
	${COGUTIL_LIBRARY}
)
SET_TESTS_PROPERTIES(PythonUtilitiesUTest
	PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")


IF (HAVE_NOSETESTS)

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
