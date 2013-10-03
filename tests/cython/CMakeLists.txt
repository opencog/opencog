INCLUDE_DIRECTORIES(
	${PYTHON_INCLUDE_PATH}
)

IF (NOSETESTS_EXECUTABLE)
	ADD_TEST(Cython ${NOSETESTS_EXECUTABLE} -vs
		${CMAKE_SOURCE_DIR}/tests/cython/)
	SET_TESTS_PROPERTIES(Cython
		PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython:${PROJECT_SOURCE_DIR}/tests/cython/agents")

ELSE (NOSETESTS_EXECUTABLE)
	MESSAGE(WARNING "Nosetests executable for testing Python bindings not found. Install with \"sudo easy_install nose\".")
ENDIF (NOSETESTS_EXECUTABLE)

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
)
SET_TESTS_PROPERTIES(PythonModuleUTest
	PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")

