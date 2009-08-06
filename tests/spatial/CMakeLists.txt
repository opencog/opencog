# This test is very slow...
#IF(CMAKE_COMPILER_IS_GNUCXX)
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O")
#ENDIF(CMAKE_COMPILER_IS_GNUCXX)

ADD_CXXTEST(LocalSpaceMap2DUTest)
TARGET_LINK_LIBRARIES(LocalSpaceMap2DUTest
    SpaceMap 
    util
)

ADD_CXXTEST(VisibilityMapUTest)
TARGET_LINK_LIBRARIES(VisibilityMapUTest
    SpaceMap
    util
)

ADD_CXXTEST(HPASearchUTest)
TARGET_LINK_LIBRARIES(HPASearchUTest
    SpaceMap 
    util
    HPASearch
)

ADD_CXXTEST(MathUTest)
TARGET_LINK_LIBRARIES(MathUTest
    SpaceMap
    util
)
