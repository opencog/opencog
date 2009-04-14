ADD_LIBRARY (LearningServerMessages
    LearnMessage
    SchemaMessage
    RewardMessage
    LSCmdMessage
    TrySchemaMessage
    StopLearningMessage
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
