INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/guile
	${PROJECT_SOURCE_DIR}/opencog/util
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/guile
	${PROJECT_BINARY_DIR}/opencog/util
)

# list libsureal as a dependency, so that it gets rebuilt, if needed,
# when building this test.
LINK_LIBRARIES(
	sureal
	${NO_AS_NEEDED}
	${ATOMSPACE_LIBRARIES}
	${COGUTIL_LIBRARY}
)

ADD_CXXTEST(SuRealUTest)
