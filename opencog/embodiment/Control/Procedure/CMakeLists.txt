ADD_LIBRARY (Procedure
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
    BuiltIn/PetActionSchema
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
    util
    ${XERCESC_LIBRARIES}
    AvatarComboVocabulary
)

ADD_EXECUTABLE (combo_shell ComboPAIExecutable)
TARGET_LINK_LIBRARIES (combo_shell
    Procedure
    WorldWrapper
    util
    ${XERCESC_LIBRARIES}
)
