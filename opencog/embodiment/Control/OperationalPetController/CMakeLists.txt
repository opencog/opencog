ADD_LIBRARY (OpcLib 
    Pet
    OPC
    PetMessageSender
    PVPActionPlanSender
    ImportanceDecayTask
    ActionSelectionTask
    SleepTask
    PetInterfaceUpdaterTask
    MockOpcHCTest
    HCTestTask
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
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PredicateUpdaters
    ${OPENCOG_SOURCE_DIR}/include/xerces
    ${OPENCOG_SOURCE_DIR}/include/lua
    ${OPENCOG_SOURCE_DIR}/include/luabind
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/LearningServerMessages
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
    ${OPENCOG_SOURCE_DIR}/lib/linux/xerces
    ${OPENCOG_SOURCE_DIR}/lib/linux/lua
    ${OPENCOG_SOURCE_DIR}/lib/linux/luabind
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/LearningServerMessages
)

TARGET_LINK_LIBRARIES (OpcLib
    lua5.1
    luabind
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
    ${OPENCOG_LIBRARIES}
)

ADD_EXECUTABLE (opc OPCExecutable.cc)
TARGET_LINK_LIBRARIES (opc
    OpcLib
    comboreduct
    PetComboVocabulary
)

ADD_EXECUTABLE (opcHcTester MockOpcHCTestExec.cc)
TARGET_LINK_LIBRARIES (opcHcTester
    OpcLib
    comboreduct
    PetComboVocabulary
)
