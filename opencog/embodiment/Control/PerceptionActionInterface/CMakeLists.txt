INCLUDE_DIRECTORIES (
	${PETAVERSE_SOURCE_DIR}/src/AtomSpaceExtensions
	${PETAVERSE_SOURCE_DIR}/src/Spatial
	${PETAVERSE_SOURCE_DIR}/src/Spatial/Math
	${PETAVERSE_SOURCE_DIR}/src/Control
	${PETAVERSE_SOURCE_DIR}/src/Control/Predavese
	${PETAVERSE_SOURCE_DIR}/include/xerces
)

LINK_DIRECTORIES (
	${PETAVERSE_BINARY_DIR}/src/AtomSpaceExtensions
	${PETAVERSE_BINARY_DIR}/src/Control
	${PETAVERSE_BINARY_DIR}/src/Control/Predavese
	${PETAVERSE_SOURCE_DIR}/lib/linux/xerces
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
	${LADSUTIL_LIBRARY} 
	xerces-c 
	${Boost_DATE_TIME_LIBRARY} 
	${Boost_REGEX_LIBRARY}
	${OPENCOG_LIBRARIES}
)
