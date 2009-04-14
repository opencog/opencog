# This test is very slow...
#IF(CMAKE_COMPILER_IS_GNUCXX)
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O")
#ENDIF(CMAKE_COMPILER_IS_GNUCXX)

ADD_CXXTEST(LocalSpaceMap2DUTest)
TARGET_LINK_LIBRARIES(LocalSpaceMap2DUTest
    SpaceMap 
    Control
    util
)

ADD_CXXTEST(VisibilityMapUTest)
TARGET_LINK_LIBRARIES(VisibilityMapUTest
    SpaceMap
    Control
    util
)

ADD_CXXTEST(HPASearchUTest)
TARGET_LINK_LIBRARIES(HPASearchUTest
    SpaceMap 
    Control
    util
    HPASearch
)

ADD_CXXTEST(MathUTest)
TARGET_LINK_LIBRARIES(MathUTest
    SpaceMap
    Control
    util
)
