IF(HAVE_CYTHON AND PYTHONINTERP_FOUND)
    IF (NOSETESTS_EXECUTABLE)
       # Attempt to make
       ADD_TEST(AllPythonModuleTests ${NOSETESTS_EXECUTABLE} -vs --with-doctest
           ${CMAKE_SOURCE_DIR}/tests/python/)
        SET_TESTS_PROPERTIES(AllPythonModuleTests
            PROPERTIES 
		ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython:${PROJECT_SOURCE_DIR}/opencog/python;PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR};PROJECT_BINARY_DIR=${PROJECT_BINARY_DIR}")
    ELSE (NOSETESTS_EXECUTABLE)
       MESSAGE(WARNING "Nosetests executable for testing Python modules not found. Install with \"sudo easy_install nose\".")
    ENDIF (NOSETESTS_EXECUTABLE)

ENDIF(HAVE_CYTHON AND PYTHONINTERP_FOUND)


