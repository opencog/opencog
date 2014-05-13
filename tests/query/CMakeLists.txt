INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/query
	${PROJECT_SOURCE_DIR}/opencog/util
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/opencog/nlp/types
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/query
	${PROJECT_BINARY_DIR}/opencog/util
)

LINK_LIBRARIES(
	atomspace
	nlp-types
	query
	server
)

# We want to run the tests in the order below;
# Each test gets progressively more complex, and exercises
# features that the later tests depend on.
ADD_CXXTEST(PatternUTest)
ADD_CXXTEST(StackUTest)
ADD_CXXTEST(BigPatternUTest)
ADD_CXXTEST(BiggerPatternUTest)
ADD_CXXTEST(LoopPatternUTest)
ADD_CXXTEST(BooleanUTest)
ADD_CXXTEST(Boolean2NotUTest)

# Its a *lot* easier to write scheme, than to write C++ code!
IF (HAVE_GUILE)
	ADD_CXXTEST(ImplicationUTest)
	ADD_CXXTEST(ExecutionUTest)
	ADD_CXXTEST(BuggyStackUTest)
	ADD_CXXTEST(VarTypeNotUTest)
	ADD_CXXTEST(BuggyNotUTest)
	ADD_CXXTEST(MatchLinkUTest)
	ADD_CXXTEST(UnorderedUTest)
	ADD_CXXTEST(StackMoreUTest)
	ADD_CXXTEST(EinsteinUTest)
        ADD_CXXTEST(SingleUTest)

	TARGET_LINK_LIBRARIES(VarTypeNotUTest
		cogutil
	)
ENDIF (HAVE_GUILE)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/query/buggy-stack.scm
    ${PROJECT_BINARY_DIR}/tests/query/buggy-stack.scm)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/query/var-type-not.scm
    ${PROJECT_BINARY_DIR}/tests/query/var-type-not.scm)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/query/match-link.scm
    ${PROJECT_BINARY_DIR}/tests/query/match-link.scm)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/query/unordered.scm
    ${PROJECT_BINARY_DIR}/tests/query/unordered.scm)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/query/unordered-more.scm
    ${PROJECT_BINARY_DIR}/tests/query/unordered-more.scm)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/query/stackmore-o-o.scm
    ${PROJECT_BINARY_DIR}/tests/query/stackmore-o-o.scm)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/query/stackmore-o-u.scm
    ${PROJECT_BINARY_DIR}/tests/query/stackmore-o-u.scm)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/query/stackmore-u-o.scm
    ${PROJECT_BINARY_DIR}/tests/query/stackmore-u-o.scm)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/query/stackmore-u-u.scm
    ${PROJECT_BINARY_DIR}/tests/query/stackmore-u-u.scm)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/query/single.scm
    ${PROJECT_BINARY_DIR}/tests/query/single.scm)
