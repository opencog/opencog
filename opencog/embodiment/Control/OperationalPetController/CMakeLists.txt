ADD_LIBRARY (OpcLib 
    Pet
    OPC
    PetMessageSender
    PVPActionPlanSender
    ProcedureInterpreterAgent
    ImportanceDecayAgent
    ActionSelectionAgent
    SleepAgent
    PetInterfaceUpdaterAgent
    MockOpcHCTest
    HCTestAgent
    RuleEngine
    #NewRuleEngine
    RuleEngineUtil
    #NewRuleEngineUtil
    RuleEngineLearnedTricksHandler
    SchemaRunner
    ScavengerHuntAgentModeHandler
    DefaultAgentModeHandler
    LearningAgentModeHandler
)

INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PredicateUpdaters
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/LearningServerMessages
    ${OPENCOG_SOURCE_DIR}/opencog
    ${XERCESC_INCLUDE_DIR}
    ${LUA51_INCLUDE_DIR}
    ${LUABIND_INCLUDE_DIR}
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PredicateUpdaters
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/LearningServerMessages
    ${OPENCOG_BINARY_DIR}/opencog/util
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
)

TARGET_LINK_LIBRARIES (OpcLib
    ${LUA51_LIBRARIES}
    ${LUABIND_LIBRARIES}
    ${XERCESC_LIBRARIES}
    PredicateUpdaters
    PAI
    MessagingSystem
    Procedure
)

IF(COMMAND cmake_policy)
      CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

ADD_EXECUTABLE (OpcTester OpcTester.cc)
TARGET_LINK_LIBRARIES (OpcTester
    OpcLib
    comboreduct
    PetComboVocabulary
    AtomSpaceExtensions
    ImaginaryLife
)

ADD_EXECUTABLE (opc OPCExecutable.cc)
TARGET_LINK_LIBRARIES (opc
    OpcLib
    comboreduct
    PetComboVocabulary
    ImaginaryLife
)

ADD_EXECUTABLE (opcHcTester MockOpcHCTestExec.cc)
TARGET_LINK_LIBRARIES (opcHcTester
    OpcLib
    comboreduct
    PetComboVocabulary
    ImaginaryLife
)
