INCLUDE_DIRECTORIES(
    ${PYTHON_INCLUDE_PATH}
)

IF(HAVE_CYTHON AND PYTHONINTERP_FOUND)
    IF (NOSETESTS_EXECUTABLE)
        ADD_TEST(CythonAtomSpace ${NOSETESTS_EXECUTABLE} -vs
            ${CMAKE_SOURCE_DIR}/tests/cython/test_atomspace.py)
        SET_TESTS_PROPERTIES(CythonAtomSpace
            PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")

        ADD_TEST(CythonAgentFinder ${NOSETESTS_EXECUTABLE} -vs
            ${CMAKE_SOURCE_DIR}/tests/cython/test_agent_finder.py)
        SET_TESTS_PROPERTIES(CythonAgentFinder 
            PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")
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
        util
        server
    )
    SET_TESTS_PROPERTIES(PythonModuleUTest
        PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")

ENDIF(HAVE_CYTHON AND PYTHONINTERP_FOUND)


