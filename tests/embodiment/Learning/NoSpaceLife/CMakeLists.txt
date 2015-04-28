
# At this time, the  NoSpaceLifeUTest test does not compile,
# because of the 2D spacemap ->3D transition.
IF(0)

ADD_CXXTEST(NoSpaceLifeUTest)
TARGET_LINK_LIBRARIES(NoSpaceLifeUTest
	behavior
	ImaginaryLife
	comboreduct
	cogutil
	${PROJECT_LIBRARIES}
)

ENDIF(0)
