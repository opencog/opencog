INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/query
	${PROJECT_SOURCE_DIR}/opencog/util	
	${PROJECT_SOURCE_DIR}/opencog/reasoning/RuleEngine/rule-engine-src
	${PROJECT_SOURCE_DIR}/opencog/reasoning/RuleEngine/rule-engine-src/pln/chaining
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/query
	${PROJECT_BINARY_DIR}/opencog/util
	${PROJECT_BINARY_DIR}/opencog/reasoning/RuleEngine/rule-engine-src
)

LINK_LIBRARIES(
	atomspace
	query
	server
	ruleengine
)

IF (HAVE_GUILE)
	ADD_CXXTEST(RuleUTest)
	ADD_CXXTEST(ForwardChainerUTest)
ENDIF (HAVE_GUILE)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/reasoning/RuleEngine/simple-assertions.scm
    ${PROJECT_BINARY_DIR}/tests/reasoning/RuleEngine/simple-assertions.scm)