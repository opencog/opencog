INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/RewritingRules
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/RewritingRules
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PerceptionActionInterface
)

ADD_LIBRARY(Filter
	EntropyFilter
	ActionFilter
	EntityRelevanceFilter
)

TARGET_LINK_LIBRARIES(Filter
	PAI
	util
	comboreduct
	PetComboVocabulary
	behavior
	RewritingRules
	WorldWrapper
	AtomSpaceExtensions
	${OPENCOG_LIBRARIES}
)
