INCLUDE_DIRECTORIES (
	${CMAKE_BINARY_DIR}
	${TBB_INCLUDE_DIR}
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/server
	${TBB_INCLUDE_DIR}
	${TBB_LIBRARY_DIRS}
)

LINK_LIBRARIES(
	${ATOMSPACE_LIBRARIES}
	server
	zmq
	tbb
)

ADD_CXXTEST(AtomSpacePublisherModuleUTest)

TARGET_LINK_LIBRARIES(AtomSpacePublisherModuleUTest
	atomspacepublishermodule
)

SET_TESTS_PROPERTIES(AtomSpacePublisherModuleUTest
	PROPERTIES ENVIRONMENT
	"PYTHONPATH=/usr/local/share/opencog/python"
)
