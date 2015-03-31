IF (HAVE_NOSETESTS)

    # Basic test, just does the cython wrapper for opencog/util
    ADD_TEST(PLNTest ${NOSETESTS_EXECUTABLE} -vs
        ${CMAKE_SOURCE_DIR}/tests/pln)
    SET_TESTS_PROPERTIES(PLNTest
        PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython:${PROJECT_SOURCE_DIR}/opencog/python:${PROJECT_SOURCE_DIR}/opencog/nlp")

ENDIF (HAVE_NOSETESTS)
