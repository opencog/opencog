INCLUDE_DIRECTORIES(
    ${PYTHON_INCLUDE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# At this time, the AvaterUTest does not compile,
# because of the 2D spacemap ->3D transition.
IF(0)
#ADD_CXXTEST(AvatarUTest)
TARGET_LINK_LIBRARIES(AvatarUTest
	oac
	Procedure
	behavior
	PAI
	MessagingSystem
	Control
	comboreduct
	cogutil
	pthread
	${Boost_DATE_TIME_LIBRARY}
	${PROJECT_LIBRARIES}
	${Boost_THREAD_LIBRARY}
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(AvatarUTest winmm)
ENDIF(WIN32)
ENDIF(0)

# Can't run the Psi*AgentUTest tests without loading the OAC stuff
# first, and loading the OAC requires a tclsh being in the search path.
# (Well, we can run the tests, but they will fail...)
IF (TCL_TCLSH)
	GET_FILENAME_COMPONENT(MAKE_DIST_BIN_DIR ${CMAKE_BINARY_DIR} NAME)
	ADD_CUSTOM_TARGET(DistributeOACStuff
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Scaffold ${CMAKE_BINARY_DIR}/dist

		COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR}/scripts/embodiment ./make_distribution  ${MAKE_DIST_BIN_DIR} Embodiment
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_BINARY_DIR}/dist/Embodiment ${CMAKE_CURRENT_BINARY_DIR}

		# The following dependencies must be built first, because the
		# scripts/embodiment/make_distribution script tries to copy them.
		# (see lines 36:42 of that script, starting with $binDirH/opencog/...
		DEPENDS ${CMAKE_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem/router
		DEPENDS ${CMAKE_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem/spawner
		DEPENDS ${CMAKE_BINARY_DIR}/opencog/embodiment/Control/OperationalAvatarController/oac-exec
		DEPENDS ${CMAKE_BINARY_DIR}/opencog/embodiment/Control/OperationalAvatarController/opcHcTester
		DEPENDS ${CMAKE_BINARY_DIR}/opencog/embodiment/Control/Procedure/combo_shell
		DEPENDS ${CMAKE_BINARY_DIR}/opencog/embodiment/Control/Procedure/combo_shell_stdio
		DEPENDS ${CMAKE_BINARY_DIR}/opencog/embodiment/Learning/LearningServer/learningServer
		DEPENDS ${CMAKE_BINARY_DIR}/opencog/embodiment/AutomatedSystemTest/pbTest
	)

	ADD_LIBRARY(oacmock
		OACMock
	)

	SET(OAC_MOCK_LINK_LIBS
        oacmock
		oac
		Procedure
		behavior
		PAI
		MessagingSystem
		Control
		comboreduct
		cogutil
		pthread
		${Boost_DATE_TIME_LIBRARY}
		${PROJECT_LIBRARIES}
		${Boost_THREAD_LIBRARY}
        
	)

    TARGET_LINK_LIBRARIES(${OAC_MOCK_LINK_LIBS})
    
	ADD_DEPENDENCIES(oacmock DistributeOACStuff)
    
#ADD_CXXTEST(PsiDemandUpdaterAgentUTest)
#	TARGET_LINK_LIBRARIES(PsiDemandUpdaterAgentUTest
#		oacmock
#	)

#ADD_CXXTEST(PsiModulatorUpdaterAgentUTest)
#	TARGET_LINK_LIBRARIES(PsiModulatorUpdaterAgentUTest
#		oacmock
#	)

#ADD_CXXTEST(PsiFeelingUpdaterAgentUTest)
#	TARGET_LINK_LIBRARIES(PsiFeelingUpdaterAgentUTest
#		oacmock
#	)

#ADD_CXXTEST(PsiActionSelectionAgentUTest)    

    SET(PASA_TEST_LINK_LIBS oacmock)
    IF (HAVE_CYTHON)
        SET(PASA_TEST_LINK_LIBS
            ${PASA_TEST_LINK_LIBS}
            PythonEval
            PythonModule
            atomspace
            cogutil
            ${PYTHON_LIBRARIES})
    ENDIF (HAVE_CYTHON)

#	TARGET_LINK_LIBRARIES(PsiActionSelectionAgentUTest
#		${PASA_TEST_LINK_LIBS}
#	)
ELSE (TCL_TCLSH)
	SUMMARY_ADD("Psi*AgentUTest" "Requires TCL shell to load test data" 0)
ENDIF (TCL_TCLSH)

SUMMARY_SHOW()
