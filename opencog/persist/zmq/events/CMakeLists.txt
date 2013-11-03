INCLUDE_DIRECTORIES (
    ${CMAKE_BINARY_DIR}
)

ADD_LIBRARY (atomspacepublishermodule SHARED
    AtomSpacePublisherModule
)

TARGET_LINK_LIBRARIES(atomspacepublishermodule
    zmq
)

