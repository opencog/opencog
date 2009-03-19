ADD_LIBRARY(behavior
    BDTracker
    BE
    PAIWorldProvider
    SpaceServerWorldProvider
    PredicateHandleSet
    BehaviorCategory
    BehaviorDescriptionMatcher
    CompositeBehaviorDescription
    EvaluationLinkSimilarityEvaluator
    BDRetriever
)

INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/opencog
    ${OPENCOG_SOURCE_DIR}/opencog/atomspace
    ${OPENCOG_SOURCE_DIR}/include/xerces
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_BINARY_DIR}/opencog/util
    ${OPENCOG_BINARY_DIR}/opencog/atomspace
    ${OPENCOG_SOURCE_DIR}/lib/linux/xerces
)

TARGET_LINK_LIBRARIES (behavior
	PAI
	util
	xerces-c
	MessagingSystem
	AtomSpaceExtensions
	atomspace
)
