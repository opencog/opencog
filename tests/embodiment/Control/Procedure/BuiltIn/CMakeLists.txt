INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController/PredicateUpdaters
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/tests/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/tests/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure/BuiltIn
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog
)

SET(CMAKE_BUILD_TYPE Debug)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/util
)

ADD_CXXTEST(PetActionSchemaUTest)
TARGET_LINK_LIBRARIES(PetActionSchemaUTest
    Procedure
    behavior
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(PetActionSchemaUTest winmm)
ENDIF(WIN32)

#ADD_CXXTEST(PetRandomSchemaUTest PopulateAtomSpace.cc)
#TARGET_LINK_LIBRARIES(PetRandomSchemaUTest
#    OpcLib
#    Procedure
#    behavior
#)

#IF(WIN32)
#    TARGET_LINK_LIBRARIES(PetRandomSchemaUTest winmm)
#ENDIF(WIN32)

#ADD_CXXTEST(BehaviorSchemaUTest)
#TARGET_LINK_LIBRARIES(BehaviorSchemaUTest
#    OpcLib
#    PAI
#    Procedure
#    Predavese
#    MessagingSystem
#    Control
#    ${COMBOREDUCT_LIBRARY}
#    ${OPENCOG_LIBRARIES}
#    ${LADSUTIL_LIBRARY}
#    xerces-c
#    boost
#    date
#    time
#    behavior
#)

#IF(WIN32)
#    TARGET_LINK_LIBRARIES(BehaviorSchemaUTest winmm)
#ENDIF(WIN32)
