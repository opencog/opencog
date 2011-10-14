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
    AvatarComboVocabulary
    comboreduct
    util
    xml
    AtomSpaceExtensions
    atomspace
    ${PROJECT_LIBRARIES}
)
