ADD_LIBRARY (FitnessEstimator
	NoSpaceLifeFitnessEstimator
	DistortedComboSize
	SizePenalty
)

INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/NoSpaceLife
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog
    ${XERCESC_INCLUDE_DIR}
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/NoSpaceLife
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/util
)

TARGET_LINK_LIBRARIES (FitnessEstimator
	util
	${XERCESC_LIBRARIES}
	comboreduct
	behavior
	WorldWrapper
	AtomSpaceExtensions
)
