INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure/BuiltIn
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/include/xerces
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/lib/linux/xerces
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
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
