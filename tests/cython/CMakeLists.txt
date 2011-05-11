INCLUDE_DIRECTORIES(
    ${PYTHON_INCLUDE_PATH}
)

IF(PYTHONINTERP_FOUND)
    ADD_TEST(CythonAtomSpace nosetests -vs
        ${CMAKE_SOURCE_DIR}/tests/cython/test_atomspace.py)
    SET_TESTS_PROPERTIES(CythonAtomSpace
        PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")

    ADD_TEST(CythonAgentFinder nosetests -vs
        ${CMAKE_SOURCE_DIR}/tests/cython/test_agent_finder.py)
    SET_TESTS_PROPERTIES(CythonAgentFinder 
        PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")

    ADD_CXXTEST(PythonModuleUTest)
ENDIF(PYTHONINTERP_FOUND)

ADD_DEPENDENCIES(PythonModuleUTest
    PythonModule
)

TARGET_LINK_LIBRARIES(PythonModuleUTest
    util
    server
)
