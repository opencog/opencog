INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/src/atomspace
	${PROJECT_SOURCE_DIR}/src/persist/file
	${PROJECT_SOURCE_DIR}/src/persist/sql
	${PROJECT_SOURCE_DIR}/src/util
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/src/atomspace
	${PROJECT_BINARY_DIR}/src/persist
	${PROJECT_BINARY_DIR}/src/util
)

LINK_LIBRARIES(
	atomspace
	persist
	server
	${Boost_SIGNALS_LIBRARY}
)

ADD_CXXTEST(BasicSaveUTest)

