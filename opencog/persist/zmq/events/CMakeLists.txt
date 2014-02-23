INCLUDE_DIRECTORIES (
        ${CMAKE_BINARY_DIR}
)

LINK_DIRECTORIES(
        ${PROJECT_BINARY_DIR}/opencog/atomspace
        ${PROJECT_BINARY_DIR}/opencog/server
)

ADD_LIBRARY (atomspacepublishermodule SHARED
    AtomSpacePublisherModule
)

LINK_LIBRARIES(
        server
)

TARGET_LINK_LIBRARIES(atomspacepublishermodule
    zmq
)
