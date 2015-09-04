
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
        SpaceMapUtil
        PathFinder3D
	MessagingSystem
	AvatarComboVocabulary
)
