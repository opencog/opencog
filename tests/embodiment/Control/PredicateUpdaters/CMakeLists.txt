INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/tests/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PredicateUpdaters
	${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_BINARY_DIR}/opencog/util
)

ADD_CXXTEST(PredicatesUpdaterUTest)
TARGET_LINK_LIBRARIES(PredicatesUpdaterUTest 
	PredicateUpdaters 
	PAI 
	xerces-c
	${OPENCOG_LIBRARIES}
)
IF(WIN32)
TARGET_LINK_LIBRARIES(PredicatesUpdaterUTest 
	PredicateUpdaters 
	PAI 
	xerces-c 
	winmm
	${OPENCOG_LIBRARIES}
)
ENDIF(WIN32)

