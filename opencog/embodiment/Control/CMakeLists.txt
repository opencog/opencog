ADD_SUBDIRECTORY (MessagingSystem)
ADD_SUBDIRECTORY (Language)
ADD_SUBDIRECTORY (OperationalAvatarController)
ADD_SUBDIRECTORY (PerceptionActionInterface)
ADD_SUBDIRECTORY (Procedure)
ADD_SUBDIRECTORY (PredicateUpdaters)

ADD_LIBRARY (Control SHARED
	SpaceTime
	EmbodimentConfig
	LoggerFactory
)

TARGET_LINK_LIBRARIES(Control
	MessagingSystem
)
