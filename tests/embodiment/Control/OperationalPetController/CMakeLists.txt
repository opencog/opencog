ADD_CXXTEST(PetUTest)
TARGET_LINK_LIBRARIES(PetUTest 
	OpcLib 
	Procedure 
	behavior 
	PAI 
	MessagingSystem 
	Control 
	comboreduct
	util 
	pthread 
	${Boost_DATE_TIME_LIBRARY}
	${XERCESC_LIBRARIES}
	${OPENCOG_LIBRARIES}
	${Boost_THREAD_LIBRARY} 
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(PetUTest winmm)
ENDIF(WIN32)

IF (GUILE_FOUND)
	ADD_CXXTEST(LanguageComprehensionUTest)
	TARGET_LINK_LIBRARIES(LanguageComprehensionUTest 
		OpcLib
		behavior
		${XERCESC_LIBRARIES}
		${Boost_THREAD_LIBRARY}
	)
	
	IF(WIN32)
		TARGET_LINK_LIBRARIES(LanguageComprehensionUTest winmm)
	ENDIF(WIN32)
ENDIF (GUILE_FOUND)

