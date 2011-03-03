ADD_CXXTEST(AvatarUTest)
TARGET_LINK_LIBRARIES(AvatarUTest 
	oac 
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
	${PROJECT_LIBRARIES}
	${Boost_THREAD_LIBRARY} 
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(AvatarUTest winmm)
ENDIF(WIN32)

IF (GUILE_FOUND)
	ADD_CXXTEST(LanguageComprehensionUTest)
	TARGET_LINK_LIBRARIES(LanguageComprehensionUTest 
		oac
		behavior
		${XERCESC_LIBRARIES}
		${Boost_THREAD_LIBRARY}
	)

	IF(WIN32)
		TARGET_LINK_LIBRARIES(LanguageComprehensionUTest winmm)
	ENDIF(WIN32)
ENDIF (GUILE_FOUND)


