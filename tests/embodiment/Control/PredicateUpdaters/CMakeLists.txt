
# At this time, this test does not compile,
# because of the 2D spacemap ->3D transition.
IF(0)

ADD_CXXTEST(PredicatesUpdaterUTest)
TARGET_LINK_LIBRARIES(PredicatesUpdaterUTest 
	atomspace
	server
	PredicateUpdaters 
	PAI 
	${PROJECT_LIBRARIES}
)

IF(WIN32)
TARGET_LINK_LIBRARIES(PredicatesUpdaterUTest 
	winmm
)
ENDIF(WIN32)

ENDIF(0)
