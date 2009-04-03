ADD_LIBRARY(PetaverseMOSES
	moses-learning
)

INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/hillclimbing
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/FitnessEstimator 
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/Filter 
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/ImitationLearningAPI
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/RewritingRules
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_SOURCE_DIR}/opencog/learning/moses
    ${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES(
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/hillclimbing
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/FitnessEstimator
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/RewritingRules
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/Filter
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/util
    ${OPENCOG_BINARY_DIR}/opencog/learning/moses
)

TARGET_LINK_LIBRARIES(PetaverseMOSES
    moses
    util
    comboreduct
    FitnessEstimator
    PetComboVocabulary
)
