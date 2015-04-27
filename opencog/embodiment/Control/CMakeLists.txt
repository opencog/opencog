ADD_SUBDIRECTORY (MessagingSystem)
ADD_SUBDIRECTORY (Language)
ADD_SUBDIRECTORY (PerceptionActionInterface)
ADD_SUBDIRECTORY (PredicateUpdaters)

IF(HAVE_MOSES)
	ADD_SUBDIRECTORY (OperationalAvatarController)
	ADD_SUBDIRECTORY (Procedure)
ENDIF(HAVE_MOSES)

ADD_LIBRARY (Control SHARED
	EmbodimentConfig
	LoggerFactory
)

