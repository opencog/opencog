INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/opencog
	${XERCESC_INCLUDE_DIR}
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_BINARY_DIR}/opencog/util
)

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
	Predavese 
	Control 
	util 
	${XERCESC_LIBRARIES}
	${Boost_DATE_TIME_LIBRARY} 
	${Boost_REGEX_LIBRARY}
)
