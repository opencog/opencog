INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/persist/file
	${PROJECT_SOURCE_DIR}/opencog/persist/sql
	${PROJECT_SOURCE_DIR}/opencog/util
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/persist
	${PROJECT_BINARY_DIR}/opencog/util
)

LINK_LIBRARIES(
	atomspace
	persist
	server
	${Boost_SIGNALS_LIBRARY}
)

# These two tests currrently work and pass, as of 10 December 2013
# They are commented out only because they will fail for anyone who
# hasn't corrrectly configured their SQL setup.  To run these tests,
# see the README file, and uncomment the below.
#
# ADD_CXXTEST(BasicSaveUTest)
# ADD_CXXTEST(PersistUTest)

