ADD_LIBRARY (oac 
    Pet
    OAC
    PetMessageSender
    PVPActionPlanSender
    ProcedureInterpreterAgent
    ImportanceDecayAgent
    ActionSelectionAgent
    EntityExperienceAgent
    PsiModulatorUpdaterAgent
    PsiDemandUpdaterAgent
    SleepAgent
    MockOpcHCTest
    HCTestAgent
    RuleEngine
    RuleEngineUtil
    RuleEngineLearnedTricksHandler
    SchemaRunner
    BaseAgentModeHandler
    ScavengerHuntAgentModeHandler
    DefaultAgentModeHandler
    LearningAgentModeHandler
    LanguageComprehensionCore
    LanguageComprehensionFrames
    LanguageComprehensionDialogController
    FramesToRelexRuleEngine
    NLGenClient
    OutputRelex
)

INCLUDE_DIRECTORIES (
    ${XERCESC_INCLUDE_DIR}
    ${LUA_INCLUDE_DIR}
    ${LUABIND_INCLUDE_DIR}
)

TARGET_LINK_LIBRARIES (oac
    ${LUA_LIBRARIES}
    ${LUABIND_LIBRARIES}
    ${XERCESC_LIBRARIES}
    PredicateUpdaters
    PAI
    MessagingSystem
    Procedure
    savable
)

IF(COMMAND cmake_policy)
      CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

ADD_EXECUTABLE (OACTester OACTester.cc)
TARGET_LINK_LIBRARIES (OACTester
    oac
    comboreduct
    PetComboVocabulary
    AtomSpaceExtensions
    ImaginaryLife
    ${Boost_THREAD_LIBRARY}
)

ADD_EXECUTABLE (opc OACExecutable.cc)
TARGET_LINK_LIBRARIES (opc
    oac
    comboreduct
    PetComboVocabulary
    ImaginaryLife
    ${Boost_THREAD_LIBRARY} 
)

ADD_EXECUTABLE (opcHcTester MockOpcHCTestExec.cc)
TARGET_LINK_LIBRARIES (opcHcTester
    oac
    comboreduct
    PetComboVocabulary
    ImaginaryLife
)
