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

ADD_LIBRARY (atomspacepublishermodule SHARED
    AtomSpacePublisherModule
)

LINK_LIBRARIES(
        server
)

TARGET_LINK_LIBRARIES(atomspacepublishermodule
    zmq
    json_spirit
    tbb
)
