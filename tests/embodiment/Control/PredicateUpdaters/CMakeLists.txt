ADD_CXXTEST(PredicatesUpdaterUTest)
TARGET_LINK_LIBRARIES(PredicatesUpdaterUTest 
	PredicateUpdaters 
	PAI 
	${XERCESC_LIBRARIES}
	${OPENCOG_LIBRARIES}
)
IF(WIN32)
TARGET_LINK_LIBRARIES(PredicatesUpdaterUTest 
	PredicateUpdaters 
	PAI 
	winmm
	${XERCESC_LIBRARIES}
	${OPENCOG_LIBRARIES}
)
ENDIF(WIN32)

