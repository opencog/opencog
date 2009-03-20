CMAKE_POLICY(SET CMP0005 OLD)
ADD_DEFINITIONS (-DPVP_XML_FILE_PATH=\\"${CMAKE_CURRENT_SOURCE_DIR}\\")

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
        ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PredicateUpdaters
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/test/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
	${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/util
	${OPENCOG_BINARY_DIR}/opencog/comboreduct
	${OPENCOG_BINARY_DIR}
)

ADD_CXXTEST(PAIUTest)
TARGET_LINK_LIBRARIES(PAIUTest PAI behavior)
IF(WIN32)
    TARGET_LINK_LIBRARIES(PAIUTest winmm)
ENDIF(WIN32)

ADD_EXECUTABLE(processPVPMessage ProcessPVPMessage.cc)
TARGET_LINK_LIBRARIES(processPVPMessage 
	PredicateUpdaters 
	PAI
	behavior
	${OPENCOG_LIBRARIES}
)

ADD_CXXTEST(ActionPlanUTest)
TARGET_LINK_LIBRARIES(ActionPlanUTest PAI util comboreduct)
IF(WIN32)
    TARGET_LINK_LIBRARIES(ActionPlanTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(ActionTypeUTest)
TARGET_LINK_LIBRARIES(ActionTypeUTest PAI util comboreduct)
IF(WIN32)
    TARGET_LINK_LIBRARIES(ActionTypeTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(ActionParamTypeUTest)
TARGET_LINK_LIBRARIES(ActionParamTypeUTest PAI util comboreduct)
IF(WIN32)
    TARGET_LINK_LIBRARIES(ActionParamTypeTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(PetActionUTest)
TARGET_LINK_LIBRARIES(PetActionUTest PAI util comboreduct)
IF(WIN32)
    TARGET_LINK_LIBRARIES(PetActionUTest winmm)
ENDIF(WIN32)

