INCLUDE_DIRECTORIES (
        ${CMAKE_BINARY_DIR}
)

LINK_DIRECTORIES(
        ${PROJECT_BINARY_DIR}/opencog/atomspace
        ${PROJECT_BINARY_DIR}/opencog/server
)

LINK_LIBRARIES(
        server
        atomspace
        zmq
)

IF (HAVE_ZMQ)
        ADD_CXXTEST(AtomSpacePublisherModuleUTest)
ENDIF (HAVE_ZMQ)
