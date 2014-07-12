
# The 2D code no longer compiles, I guess its dead code.
# ADD_CXXTEST(LocalSpaceMap2DUTest)
# TARGET_LINK_LIBRARIES(LocalSpaceMap2DUTest
#     SpaceMap 
#     util
# )

LINK_LIBRARIES(
	spacetime-types
	spacetime
)


ADD_CXXTEST(VisibilityMapUTest)

ADD_CXXTEST(HPASearchUTest)
TARGET_LINK_LIBRARIES(HPASearchUTest
    HPASearch
)

ADD_CXXTEST(MathUTest)
ADD_CXXTEST(TemporalUTest)
ADD_CXXTEST(TemporalMapUTest)
ADD_CXXTEST(TemporalTableUTest)
ADD_CXXTEST(TimeServerUTest)

