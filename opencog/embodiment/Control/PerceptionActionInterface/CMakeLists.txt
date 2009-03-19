INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/opencog
	${OPENCOG_SOURCE_DIR}/include/xerces
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_BINARY_DIR}/opencog/util
	${OPENCOG_SOURCE_DIR}/lib/linux/xerces
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
	xerces-c 
	${Boost_DATE_TIME_LIBRARY} 
	${Boost_REGEX_LIBRARY}
	${OPENCOG_LIBRARIES}
)
