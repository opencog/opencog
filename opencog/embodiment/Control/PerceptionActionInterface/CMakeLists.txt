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
	PetaverseDOMParser
	PetaverseErrorHandler
)

TARGET_LINK_LIBRARIES(PAI 
	AtomSpaceExtensions
	Predavese 
	util 
	${XERCESC_LIBRARIES}
	${Boost_DATE_TIME_LIBRARY} 
	${Boost_REGEX_LIBRARY}
)

