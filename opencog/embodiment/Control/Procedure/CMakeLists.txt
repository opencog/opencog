ADD_LIBRARY (Procedure SHARED
	RunningProcedureId
	ComboProcedure
	ComboInterpreter
	RunningComboProcedure
	ComboProcedureRepository
	BuiltInProcedureRepository
	ProcedureRepository
	ComboShellServer
	ProcedureInterpreter
	RunningBuiltInProcedure
	BuiltIn/AvatarActionSchema
)

TARGET_LINK_LIBRARIES (Procedure
	WorldWrapper
)

ADD_DEPENDENCIES(Procedure
	PAI.MapInfo.pb.h)

ADD_EXECUTABLE (combo_shell_stdio ComboSTDIOExecutable)
TARGET_LINK_LIBRARIES (combo_shell_stdio
	# OAC is deprecated.  If you are working on refactoring some of the code
	# you can uncomment.
	#oac
	Procedure
	WorldWrapper
	AvatarComboVocabulary
	${COGUTIL_LIBRARY}
)
IF (HAVE_PROTOBUF)
   TARGET_LINK_LIBRARIES(combo_shell_stdio
	  ${PROTOBUF_LIBRARY}
   )
ENDIF (HAVE_PROTOBUF)

# -----------------------------------------------------

ADD_EXECUTABLE (combo_shell ComboPAIExecutable)
TARGET_LINK_LIBRARIES (combo_shell
	Procedure
	MessagingSystem
	WorldWrapper
)

IF (HAVE_PROTOBUF)
	TARGET_LINK_LIBRARIES(combo_shell
		${PROTOBUF_LIBRARY}
	)
ENDIF (HAVE_PROTOBUF)
