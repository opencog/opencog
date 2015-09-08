
ADD_LIBRARY(WorldWrapper SHARED
	WorldWrapper
	WorldWrapperUtil
	WorldWrapperUtilCache
	ShellWorldWrapper
	PAIWorldWrapper
)

TARGET_LINK_LIBRARIES(WorldWrapper
	Control
    PAI
	TangentBug
	AStar
	HPASearch
	MessagingSystem
	AvatarComboVocabulary
)
