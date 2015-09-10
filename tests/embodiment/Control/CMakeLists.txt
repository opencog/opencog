ADD_SUBDIRECTORY (MessagingSystem)
ADD_SUBDIRECTORY (PerceptionActionInterface)
ADD_SUBDIRECTORY (PredicateUpdaters)

IF(HAVE_MOSES)
	# OAC is deprecated.  If you are working on refactoring some of the code
	# you can uncomment.
	#ADD_SUBDIRECTORY (OperationalAvatarController)
	ADD_SUBDIRECTORY (Procedure)
ENDIF(HAVE_MOSES)
