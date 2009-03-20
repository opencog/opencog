INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
        ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PredicateUpdaters
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/tests/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/tests/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure/BuiltIn
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
        ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
        ${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}
	${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
	${OPENCOG_BINARY_DIR}/opencog/util
)

ADD_CXXTEST(OPCUTest)
TARGET_LINK_LIBRARIES(OPCUTest 
	OpcLib 
	Predavese 
	MessagingSystem 
	Control 
        util
	xerces-c 
	pthread 
	ssl 
	crypto 
	dl 
	${OPENCOG_LIBRARIES}
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(OPCUTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(PetUTest)
TARGET_LINK_LIBRARIES(PetUTest 
	OpcLib 
	Procedure 
	behavior 
	PAI 
	Predavese 
	MessagingSystem 
	Control 
        comboreduct
	util 
	xerces-c 
	pthread 
	ssl
	crypto 
	dl
	${Boost_DATE_TIME_LIBRARY}
	${OPENCOG_LIBRARIES}
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(PetUTest winmm)
ENDIF(WIN32)

ADD_EXECUTABLE(HttpRequestTest HttpRequestTest.cc)
TARGET_LINK_LIBRARIES(HttpRequestTest 
	pthread 
	ssl
	crypto 
        dl 
        ${CSOCKETS_LIBRARIES}
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(HttpRequestTest winmm)
ENDIF(WIN32)
