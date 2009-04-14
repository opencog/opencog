ADD_LIBRARY (Procedure
    RunningProcedureId
    ComboProcedure
    ComboInterpreter
    RunningComboProcedure
    ComboProcedureRepository
    BuiltInProcedureRepository
    ProcedureRepository
    ComboSelectProcedure
    ComboSelectProcedureRepository
    ComboSelectInterpreter
    RunningComboSelectProcedure
    ComboShellServer
    ProcedureInterpreter
    RunningBuiltInProcedure
    BuiltIn/PetActionSchema
)

TARGET_LINK_LIBRARIES (Procedure
    PAI
    MessagingSystem
    comboreduct
    WorldWrapper
    pthread
    ssl
    crypto
    dl
    VirtualWorldData
)

#SET(CMAKE_BUILD_TYPE Debug)

IF(COMMAND cmake_policy)
    CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

ADD_EXECUTABLE (combo_shell_stdio ComboSTDIOExecutable)
TARGET_LINK_LIBRARIES (combo_shell_stdio
    comboreduct
    Procedure
    behavior
    PAI
    Predavese
    Control
    util
    ${XERCESC_LIBRARIES}
    PetComboVocabulary
)

ADD_EXECUTABLE (combo_shell ComboPAIExecutable)
TARGET_LINK_LIBRARIES (combo_shell
    comboreduct
    Procedure
    behavior
    PAI
    Predavese
    MessagingSystem
    Control
    WorldWrapper
    util
    ${XERCESC_LIBRARIES}
    ssl
    crypto
    pthread
    dl
    PetComboVocabulary
)
