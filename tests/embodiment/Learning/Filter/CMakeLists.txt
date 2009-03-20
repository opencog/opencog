INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/behavior
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/Filter
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/LearningServer
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/RewritingRules
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
        ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure/BuiltIn
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/behavior
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/Filter
	${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/RewritingRules
	${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Procedure
	${OPENCOG_BINARY_DIR}/opencog/util
	${OPENCOG_BINARY_DIR}/opencog/comboreduct
)

ADD_CXXTEST(EntropyFilterUTest)
TARGET_LINK_LIBRARIES(EntropyFilterUTest
	PetComboVocabulary
	Filter
        comboreduct
        util
	${OPENCOG_LIBRARIES}
)

ADD_CXXTEST(ActionFilterUTest)
TARGET_LINK_LIBRARIES(ActionFilterUTest
	behavior
	PetComboVocabulary
	Filter
        comboreduct
        util
	${OPENCOG_LIBRARIES}
)

ADD_CXXTEST(EntityRelevanceFilterUTest)
TARGET_LINK_LIBRARIES(EntityRelevanceFilterUTest 
	Filter 
        util
	${OPENCOG_LIBRARIES}
)
IF(WIN32)
    TARGET_LINK_LIBRARIES(EntityRelevanceFilterUTest winmm)
ENDIF(WIN32)
