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
    ${XERCESC_INCLUDE_DIR}
    ${LUA51_INCLUDE_DIR}
    ${LUABIND_INCLUDE_DIR}
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
