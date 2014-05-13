INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/query
	${PROJECT_SOURCE_DIR}/opencog/util
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/query
	${PROJECT_BINARY_DIR}/opencog/util
)

LINK_LIBRARIES(
	atomspace
	query
	server
)

IF (HAVE_GUILE)
	ADD_CXXTEST(RuleUTest)
ENDIF (HAVE_GUILE)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/learning/RuleEngine/simple-assertions.scm
    ${PROJECT_BINARY_DIR}/tests/learning/RuleEngine/simple-assertions.scm)
