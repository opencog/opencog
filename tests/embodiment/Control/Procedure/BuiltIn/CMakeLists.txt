SET(CMAKE_BUILD_TYPE Debug)

ADD_CXXTEST(PetActionSchemaUTest)
TARGET_LINK_LIBRARIES(PetActionSchemaUTest
    Procedure
    behavior
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(PetActionSchemaUTest winmm)
ENDIF(WIN32)
