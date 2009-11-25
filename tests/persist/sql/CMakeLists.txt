INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/persist/file
	${OPENCOG_SOURCE_DIR}/src/persist/sql
	${OPENCOG_SOURCE_DIR}/src/util
)

LINK_DIRECTORIES(
	${OPENCOG_BINARY_DIR}/src/atomspace
	${OPENCOG_BINARY_DIR}/src/persist
	${OPENCOG_BINARY_DIR}/src/util
)

LINK_LIBRARIES(
	atomspace
	persist
	server
	${Boost_SIGNALS_LIBRARY}
)

ADD_CXXTEST(BasicSaveUTest)

