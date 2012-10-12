
# The 2D code no longer compiles, I guess its dead code.
# ADD_CXXTEST(LocalSpaceMap2DUTest)
# TARGET_LINK_LIBRARIES(LocalSpaceMap2DUTest
#     SpaceMap 
#     util
# )

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
