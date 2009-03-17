INCLUDE_DIRECTORIES (
    ${PETAVERSE_SOURCE_DIR}/src/AtomSpaceExtensions
    ${PETAVERSE_SOURCE_DIR}/src/WorldWrapper
    ${PETAVERSE_SOURCE_DIR}/src/Control
    ${PETAVERSE_SOURCE_DIR}/src/Spatial
    ${PETAVERSE_SOURCE_DIR}/src/Spatial/Math
    ${PETAVERSE_SOURCE_DIR}/src/RuleValidation/VirtualWorldData
    ${PETAVERSE_SOURCE_DIR}/src/Control/PerceptionActionInterface
    ${PETAVERSE_SOURCE_DIR}/src/Control/Predavese
    ${PETAVERSE_SOURCE_DIR}/src/Control/Procedure
    ${PETAVERSE_SOURCE_DIR}/src/Control/Procedure/BuiltIn
    ${PETAVERSE_SOURCE_DIR}/src/Control/MessagingSystem
    ${PETAVERSE_SOURCE_DIR}/include/xerces
    ${PETAVERSE_SOURCE_DIR}/src/Learning
    ${PETAVERSE_SOURCE_DIR}/src/PetComboVocabulary
    ${PETAVERSE_SOURCE_DIR}/src/Control/OperationalPetController
)

LINK_DIRECTORIES (
    ${PETAVERSE_BINARY_DIR}/src/WorldWrapper
    ${PETAVERSE_BINARY_DIR}/src/Control
    ${PETAVERSE_BINARY_DIR}/src/RuleValidation/VirtualWorldData
    ${PETAVERSE_BINARY_DIR}/src/Control/PerceptionActionInterface
    ${PETAVERSE_BINARY_DIR}/src/Control/Predavese
    ${PETAVERSE_SOURCE_DIR}/lib/linux/xerces
    ${PETAVERSE_BINARY_DIR}/src/PetComboVocabulary
)

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
    ${COMBOREDUCT_LIBRARY}
    WorldWrapper
    pthread
    ssl
    crypto
    dl
    ${OPENCOG_LIBARIES}
    VirtualWorldData
)

#SET(CMAKE_BUILD_TYPE Debug)

IF(COMMAND cmake_policy)
    CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

ADD_EXECUTABLE (combo_shell_stdio ComboSTDIOExecutable)
TARGET_LINK_LIBRARIES (combo_shell_stdio
    ${COMBOREDUCT_LIBRARY}
    Procedure
    behavior
    PAI
    Predavese
    Control
    WorldWrapper
    ${LADSUTIL_LIBRARY}
    xerces-c
    PetComboVocabulary
    ${OPENCOG_LIBRARIES}
)

ADD_EXECUTABLE (combo_shell ComboPAIExecutable)
TARGET_LINK_LIBRARIES (combo_shell
    ${COMBOREDUCT_LIBRARY}
    Procedure
    behavior
    PAI
    Predavese
    MessagingSystem
    Control
    WorldWrapper
    ${LADSUTIL_LIBRARY}
    xerces-c
    ssl
    crypto
    pthread
    dl
    PetComboVocabulary
    ${OPENCOG_LIBRARIES}
)
