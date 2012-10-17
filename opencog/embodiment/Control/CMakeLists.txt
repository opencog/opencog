ADD_SUBDIRECTORY (MessagingSystem)
ADD_SUBDIRECTORY (Language)
ADD_SUBDIRECTORY (OperationalAvatarController)
ADD_SUBDIRECTORY (PerceptionActionInterface)
ADD_SUBDIRECTORY (Procedure)
ADD_SUBDIRECTORY (PredicateUpdaters)

ADD_LIBRARY (Control SHARED
	EmbodimentConfig
	LoggerFactory
)

