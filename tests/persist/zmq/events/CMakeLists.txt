INCLUDE_DIRECTORIES (
        ${CMAKE_BINARY_DIR}
#        ${PROJECT_SOURCE_DIR}/opencog/util
#        ${PROJECT_SOURCE_DIR}/opencog/atomspace
#        ${PROJECT_SOURCE_DIR}/opencog/server
#        ${EXPAT_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
        ${PROJECT_BINARY_DIR}/opencog/atomspace
        ${PROJECT_BINARY_DIR}/opencog/server
)

#OPENCOG_ADD_ATOM_TYPES(atom_types.script
#        atom_types.h
#        atom_types.definitions
#        atom_types.inheritance
#        atom_test_types.scm)

LINK_LIBRARIES(
        server
        atomspace
        zmq
#        ${Boost_SIGNALS_LIBRARY}
)

IF (HAVE_ZMQ)
        ADD_CXXTEST(AtomSpacePublisherModuleUTest)
ENDIF (HAVE_ZMQ)
