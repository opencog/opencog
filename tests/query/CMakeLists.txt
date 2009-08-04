INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/query
	${OPENCOG_SOURCE_DIR}/src/util
)

LINK_DIRECTORIES(
	${OPENCOG_BINARY_DIR}/src/atomspace
	${OPENCOG_BINARY_DIR}/src/query
	${OPENCOG_BINARY_DIR}/src/util
)

LINK_LIBRARIES(#PatternUTest
	atomspace
	query
	server
	${Boost_SIGNALS_LIBRARY}
	${CSOCKETS_LIBRARIES}
	${OPENSSL_LIBRARIES}
)

ADD_CXXTEST(PatternUTest)
ADD_CXXTEST(StackUTest)
ADD_CXXTEST(BigPatternUTest)
ADD_CXXTEST(BiggerPatternUTest)
ADD_CXXTEST(LoopPatternUTest)
ADD_CXXTEST(BooleanUTest)
ADD_CXXTEST(Boolean2NotUTest)
ADD_CXXTEST(ImplicationUTest)
ADD_CXXTEST(ExecutionUTest)
ADD_CXXTEST(BuggyStackUTest)
