ADD_CXXTEST(PredicatesUpdaterUTest)
TARGET_LINK_LIBRARIES(PredicatesUpdaterUTest 
	PredicateUpdaters 
	PAI 
	xerces-c
	${OPENCOG_LIBRARIES}
)
IF(WIN32)
TARGET_LINK_LIBRARIES(PredicatesUpdaterUTest 
	PredicateUpdaters 
	PAI 
	xerces-c 
	winmm
	${OPENCOG_LIBRARIES}
)
ENDIF(WIN32)

