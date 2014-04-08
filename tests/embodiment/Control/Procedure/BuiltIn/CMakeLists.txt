SET(CMAKE_BUILD_TYPE Debug)

# At this time, this does not compile,
# because of the 2D spacemap ->3D transition.
IF(0)
ADD_CXXTEST(AvatarActionSchemaUTest)
TARGET_LINK_LIBRARIES(AvatarActionSchemaUTest
    Procedure
    behavior
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(AvatarActionSchemaUTest winmm)
ENDIF(WIN32)
ENDIF(0)
