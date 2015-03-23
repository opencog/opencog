
ADD_LIBRARY (LearningServerMessages SHARED
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
	Control
	AtomSpaceExtensions
	spacetime
	comboreduct
)

