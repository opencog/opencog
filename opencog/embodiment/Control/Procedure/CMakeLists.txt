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
	LearningServerMessages
	WorldWrapper
)

ADD_DEPENDENCIES(Procedure
	PAI.MapInfo.pb.h)

ADD_EXECUTABLE (combo_shell_stdio ComboSTDIOExecutable)
TARGET_LINK_LIBRARIES (combo_shell_stdio
	oac
	Procedure
	WorldWrapper
	AvatarComboVocabulary
	cogutil
)
IF (HAVE_PROTOBUF)
   TARGET_LINK_LIBRARIES(combo_shell_stdio
	  LearningServerMessages
	  ${PROTOBUF_LIBRARY}
   )
ENDIF (HAVE_PROTOBUF)

# -----------------------------------------------------

ADD_EXECUTABLE (combo_shell ComboPAIExecutable)
TARGET_LINK_LIBRARIES (combo_shell
	Procedure
	MessagingSystem
	WorldWrapper
	lslib
)

IF (HAVE_PROTOBUF)
	TARGET_LINK_LIBRARIES(combo_shell
		LearningServerMessages
		${PROTOBUF_LIBRARY}
	)
ENDIF (HAVE_PROTOBUF)

