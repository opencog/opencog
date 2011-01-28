# Run the makeDistributionForTest script before testing, 
# which would copy all sorts of .scm .combo files etc. to $CMAKE_CURRENT_BINARY_DIR
EXEC_PROGRAM("${CMAKE_CURRENT_SOURCE_DIR}/makeDistributionForUTest" 
              ARGS
              "${PROJECT_SOURCE_DIR}/scripts/embodiment/test/"
              "${CMAKE_CURRENT_BINARY_DIR}"
)

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

    ADD_CXXTEST(PsiActionSelectionAgentUTest)
    TARGET_LINK_LIBRARIES (PsiActionSelectionAgentUTest
        oac
        behavior
        PredicateUpdaters
        PAI
        MessagingSystem
        Control
        comboreduct
        util
        pthread
        Procedure
        savable
        pln
        ${LUA_LIBRARIES}
        ${LUABIND_LIBRARIES}
        ${XERCESC_LIBRARIES}   
    )

	IF(WIN32)
		TARGET_LINK_LIBRARIES(LanguageComprehensionUTest winmm)
	ENDIF(WIN32)
ENDIF (GUILE_FOUND)


