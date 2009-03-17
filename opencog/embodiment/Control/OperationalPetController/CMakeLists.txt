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
	${PETAVERSE_SOURCE_DIR}/src/AtomSpaceExtensions
    ${PETAVERSE_SOURCE_DIR}/src/WorldWrapper
    ${PETAVERSE_SOURCE_DIR}/src/Control
    ${PETAVERSE_SOURCE_DIR}/src/Spatial
    ${PETAVERSE_SOURCE_DIR}/src/Spatial/Math
    ${PETAVERSE_SOURCE_DIR}/src/RuleValidation/VirtualWorldData
    ${PETAVERSE_SOURCE_DIR}/src/Control/MessagingSystem
    ${PETAVERSE_SOURCE_DIR}/src/Control/OperationalPetController
    ${PETAVERSE_SOURCE_DIR}/src/Control/PerceptionActionInterface
    ${PETAVERSE_SOURCE_DIR}/src/Control/Predavese
    ${PETAVERSE_SOURCE_DIR}/src/Control/Procedure
    ${PETAVERSE_SOURCE_DIR}/src/Control/PredicateUpdaters
    ${PETAVERSE_SOURCE_DIR}/include/xerces
    ${PETAVERSE_SOURCE_DIR}/include/lua
    ${PETAVERSE_SOURCE_DIR}/include/luabind
    ${PETAVERSE_SOURCE_DIR}/src/Learning
    ${PETAVERSE_SOURCE_DIR}/src/PetComboVocabulary
    ${PETAVERSE_SOURCE_DIR}/src/Learning/LearningServerMessages
)

LINK_DIRECTORIES (
	${PETAVERSE_BINARY_DIR}/src/AtomSpaceExtensions
    ${PETAVERSE_BINARY_DIR}/src/WorldWrapper
    ${PETAVERSE_BINARY_DIR}/src/RuleValidation/VirtualWorldData
    ${PETAVERSE_BINARY_DIR}/src/Control
    ${PETAVERSE_BINARY_DIR}/src/Control/MessagingSystem
    ${PETAVERSE_BINARY_DIR}/src/Control/OperationalPetController
    ${PETAVERSE_BINARY_DIR}/src/Control/PerceptionActionInterface
    ${PETAVERSE_BINARY_DIR}/src/Control/Predavese
    ${PETAVERSE_BINARY_DIR}/src/Control/PredicateUpdaters
    ${PETAVERSE_SOURCE_DIR}/lib/linux/xerces
    ${PETAVERSE_SOURCE_DIR}/lib/linux/lua
    ${PETAVERSE_SOURCE_DIR}/lib/linux/luabind
    ${PETAVERSE_BINARY_DIR}/src/PetComboVocabulary
    ${PETAVERSE_BINARY_DIR}/src/Learning/LearningServerMessages
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
    ${COMBOREDUCT_LIBRARY}
    PetComboVocabulary
    AtomSpaceExtensions
    ${OPENCOG_LIBRARIES}
)

ADD_EXECUTABLE (opc OPCExecutable.cc)
TARGET_LINK_LIBRARIES (opc
    OpcLib
    ${COMBOREDUCT_LIBRARY}
    PetComboVocabulary
)

ADD_EXECUTABLE (opcHcTester MockOpcHCTestExec.cc)
TARGET_LINK_LIBRARIES (opcHcTester
    OpcLib
    ${COMBOREDUCT_LIBRARY}
    PetComboVocabulary
)
