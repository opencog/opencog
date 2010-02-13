# TODO replace python with found binary for 2.6 and skip if no python 2.6 found
IF(PYTHONINTERP_FOUND)
ADD_TEST(RESTfulTest ${PYTHON_EXECUTABLE}
    ${CMAKE_SOURCE_DIR}/tests/web/REST_test.py
    ${PROJECT_BINARY_DIR}/opencog/server/cogserver
    ${PROJECT_SOURCE_DIR}/lib/opencog.conf)
ENDIF(PYTHONINTERP_FOUND)

