# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES (
        ${CMAKE_BINARY_DIR}
)

IF (HAVE_ZMQ)

# This code has not been maintained recently and currently doesn't compile.
# Furthermore, protoc should be part of the build process so that the .pb.h and .pb.c
# files are updated dynamically, rather than being included in the repository.
#
# I'm disabling it until it is debugged and added to the unit tests or removed:
#
#ADD_LIBRARY (zmqatoms SHARED
#    ProtocolBufferSerializer
#    ZMQClient
#    ZMQMessages.pb
#    ZMQServer
#)
#
#TARGET_LINK_LIBRARIES(zmqatoms mq)

ADD_SUBDIRECTORY (events)

ENDIF (HAVE_ZMQ)

