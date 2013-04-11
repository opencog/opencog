INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/util
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/util
)

LINK_LIBRARIES(
	viterbi
	nlp-types
	atomspace
	query
	server
	${LINK_GRAMMAR_LIBRARY}
	${Boost_SIGNALS_LIBRARY}
)

# We want to run the tests in the order below;
# Each test gets progressively more complex, and exercises
# features that the later tests depend on.
ADD_CXXTEST(InitUTest)


