CMAKE_POLICY(SET CMP0005 OLD)

ADD_DEFINITIONS(-DPVP_XSD_FILE_PATH=\\"${CMAKE_CURRENT_SOURCE_DIR}/BrainProxyAxon.xsd\\")

ADD_LIBRARY (PAI
	PAIUtils
	ActionParamType
	ActionParameter
	ActionType
	PetAction
	ActionPlan
	PAI
	ActionPlanDispatcher
    EmbodimentDOMParser
    EmbodimentErrorHandler
)

SET(PAI_LINK_LIBS
	AtomSpaceExtensions
	util 
    oac
	${XERCESC_LIBRARIES}
	${Boost_DATE_TIME_LIBRARY} 
	${Boost_REGEX_LIBRARY}
)

IF (GUILE_FOUND)
   SET(PAI_LINK_LIBS smob server ${PAI_LINK_LIBS})
ENDIF (GUILE_FOUND)

TARGET_LINK_LIBRARIES(PAI ${PAI_LINK_LIBS})

