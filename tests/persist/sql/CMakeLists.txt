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

ADD_CXXTEST(BasicSaveUTest)
ADD_CXXTEST(PersistUTest)

