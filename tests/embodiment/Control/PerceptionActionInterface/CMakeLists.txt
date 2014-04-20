
ADD_DEFINITIONS (-DPVP_XML_FILE_PATH=\\"${CMAKE_CURRENT_SOURCE_DIR}\\")

# At this time, the PAIUTest does not compile,
# because of the 2D spacemap ->3D transition.
IF(0)
IF (HAVE_GUILE)
	ADD_CXXTEST(PAIUTest)
	TARGET_LINK_LIBRARIES(PAIUTest PAI behavior)
	IF(WIN32)
		TARGET_LINK_LIBRARIES(PAIUTest winmm)
	ENDIF(WIN32)
ENDIF (HAVE_GUILE)
ENDIF(0)
	
ADD_EXECUTABLE(processPVPMessage ProcessPVPMessage.cc)
TARGET_LINK_LIBRARIES(processPVPMessage 
	PredicateUpdaters 
	PAI
	behavior
	${PROJECT_LIBRARIES}
)

ADD_CXXTEST(ActionPlanUTest)
TARGET_LINK_LIBRARIES(ActionPlanUTest PAI Control comboreduct cogutil)
IF(WIN32)
    TARGET_LINK_LIBRARIES(ActionPlanTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(ActionTypeUTest)
TARGET_LINK_LIBRARIES(ActionTypeUTest PAI Control comboreduct cogutil)
IF(WIN32)
    TARGET_LINK_LIBRARIES(ActionTypeTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(ActionParamTypeUTest)
TARGET_LINK_LIBRARIES(ActionParamTypeUTest PAI Control comboreduct cogutil)
IF(WIN32)
    TARGET_LINK_LIBRARIES(ActionParamTypeTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(AvatarActionUTest)
TARGET_LINK_LIBRARIES(AvatarActionUTest PAI Control comboreduct cogutil)
IF(WIN32)
    TARGET_LINK_LIBRARIES(AvatarActionUTest winmm)
ENDIF(WIN32)

