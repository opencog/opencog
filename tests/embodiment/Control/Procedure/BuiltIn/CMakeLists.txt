SET(CMAKE_BUILD_TYPE Debug)

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
