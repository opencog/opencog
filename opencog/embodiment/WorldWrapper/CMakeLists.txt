INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/NoSpaceLife
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/TangentBug
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/NoSpaceLife
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/TangentBug
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/util    
)

ADD_LIBRARY(WorldWrapper
    WorldWrapper
    WorldWrapperUtil
    WorldWrapperUtilCache
    WorldWrapperUtilMock
    ShellWorldWrapper
    PAIWorldWrapper
    RuleValidationWorldWrapper
    NoSpaceLifeWorldWrapper
)

TARGET_LINK_LIBRARIES(WorldWrapper
    comboreduct
    util
    PAI
    TangentBug
    AStar
    HPASearch
    MessagingSystem
    ImaginaryLife
    PetComboVocabulary
    VirtualWorldData
)
