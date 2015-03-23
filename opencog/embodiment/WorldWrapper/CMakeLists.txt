
ADD_LIBRARY(WorldWrapper SHARED
	WorldWrapper
	WorldWrapperUtil
	WorldWrapperUtilCache
	ShellWorldWrapper
	PAIWorldWrapper
	NoSpaceLifeWorldWrapper
)

TARGET_LINK_LIBRARIES(WorldWrapper
	Control
	ImaginaryLife
	TangentBug
	AStar
	HPASearch
	MessagingSystem
	AvatarComboVocabulary
)
