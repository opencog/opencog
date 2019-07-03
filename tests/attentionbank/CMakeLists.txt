LINK_LIBRARIES(
	attentionval
	attentionbonk
	${ATOMSPACE_LIBRARIES}
)

ADD_CXXTEST(AttentionValueUTest)
ADD_CXXTEST(AttentionUTest)
ADD_CXXTEST(BankImplUTest)
ADD_CXXTEST(BankAsyncUTest)

ADD_CXXTEST(AttentionalFocusCBUTest)

IF (HAVE_CYTHON AND HAVE_NOSETESTS)
	ADD_TEST(AttentionBankCythonTest ${NOSETESTS_EXECUTABLE} -vs
			${CMAKE_SOURCE_DIR}/tests/attentionbank/cython/)
	SET_TESTS_PROPERTIES(AttentionBankCythonTest
						PROPERTIES  ENVIRONMENT
						"PYTHONPATH=${PROJECT_BINARY_DIR}/opencog/cython:${PROJECT_BINARY_DIR}/opencog/attentionbank:${PYTHON_ROOT}")
ENDIF (HAVE_CYTHON AND HAVE_NOSETESTS)
