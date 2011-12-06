INCLUDE_DIRECTORIES(
    ${PYTHON_INCLUDE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
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

# Can't run language comprehension without guile.
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


# Can't run this test without tclsh being in the search path.
IF (TCL_TCLSH)
	GET_FILENAME_COMPONENT(MAKE_DIST_BIN_DIR ${CMAKE_BINARY_DIR} NAME)
	ADD_CUSTOM_TARGET(DistributeOACStuff
	    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Scaffold ${CMAKE_BINARY_DIR}
	    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Scaffold ${CMAKE_CURRENT_BINARY_DIR}

	    COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR}/scripts/embodiment ./make_distribution  ${MAKE_DIST_BIN_DIR} Embodiment
	    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/dist/Embodiment ${CMAKE_BINARY_DIR}
	    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/dist/Embodiment ${CMAKE_CURRENT_BINARY_DIR}
	)
ENDIF (TCL_TCLSH)

ADD_LIBRARY(oacmock
    OACMock
)

TARGET_LINK_LIBRARIES(oacmock
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
ADD_DEPENDENCIES(oacmock DistributeOACStuff)

ADD_CXXTEST(PsiDemandUpdaterAgentUTest)
TARGET_LINK_LIBRARIES(PsiDemandUpdaterAgentUTest
    oacmock
)

ADD_CXXTEST(PsiModulatorUpdaterAgentUTest)
TARGET_LINK_LIBRARIES(PsiModulatorUpdaterAgentUTest
    oacmock
)

ADD_CXXTEST(PsiFeelingUpdaterAgentUTest)
TARGET_LINK_LIBRARIES(PsiFeelingUpdaterAgentUTest
    oacmock
)

ADD_CXXTEST(PsiActionSelectionAgentUTest)
TARGET_LINK_LIBRARIES(PsiActionSelectionAgentUTest
    oacmock
)

