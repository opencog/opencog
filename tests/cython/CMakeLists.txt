IF(PYTHONINTERP_FOUND)
    ADD_TEST(CythonBindings nosetests -vs
        ${CMAKE_SOURCE_DIR}/tests/cython/test_atomspace.py)
    set_tests_properties(CythonBindings
        PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython")
ENDIF(PYTHONINTERP_FOUND)

