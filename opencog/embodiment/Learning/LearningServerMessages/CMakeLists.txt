ADD_LIBRARY (LearningServerMessages
    LearnMessage
    SchemaMessage
    RewardMessage
    LSCmdMessage
    TrySchemaMessage
    StopLearningMessage
)

INCLUDE_DIRECTORIES (
    ${PETAVERSE_SOURCE_DIR}/src/Control/MessagingSystem
    ${PETAVERSE_SOURCE_DIR}/src/AtomSpaceExtensions
    ${PETAVERSE_SOURCE_DIR}/src/Learning
    ${PETAVERSE_SOURCE_DIR}/src/PetComboVocabulary
    ${PETAVERSE_SOURCE_DIR}/src/Spatial
    ${PETAVERSE_SOURCE_DIR}/src/Spatial/Math
)

LINK_DIRECTORIES (
    ${PETAVERSE_BINARY_DIR}/src/PetComboVocabulary
    ${PETAVERSE_BINARY_DIR}/src/AtomSpaceExtensions
    ${PETAVERSE_BINARY_DIR}/src/Spatial
)

TARGET_LINK_LIBRARIES (LearningServerMessages
    MessagingSystem
    PetComboVocabulary
    ${COMBOREDUCT_LIBRARY}
    ${LADSUTIL_LIBRARY}
    AtomSpaceExtensions
    ${OPENCOG_LIBRARIES}
)
