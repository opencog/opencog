ADD_LIBRARY (LearningServerMessages
    LearnMessage
    SchemaMessage
    RewardMessage
    LSCmdMessage
    TrySchemaMessage
    StopLearningMessage
)

INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_BINARY_DIR}/opencog/util
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/xml
    ${OPENCOG_BINARY_DIR}/opencog/atomspace
)

TARGET_LINK_LIBRARIES (LearningServerMessages
    MessagingSystem
    PetComboVocabulary
    comboreduct
    util
    xml
    AtomSpaceExtensions
    atomspace
    ${OPENCOG_LIBRARIES}
)
