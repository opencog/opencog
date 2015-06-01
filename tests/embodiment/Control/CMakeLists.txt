ADD_SUBDIRECTORY (MessagingSystem)
ADD_SUBDIRECTORY (PerceptionActionInterface)
ADD_SUBDIRECTORY (PredicateUpdaters)

IF(HAVE_MOSES)
	ADD_SUBDIRECTORY (OperationalAvatarController)
	ADD_SUBDIRECTORY (Procedure)
ENDIF(HAVE_MOSES)
