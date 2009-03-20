cmake_policy(SET CMP0005 OLD)
ADD_DEFINITIONS (-DPVP_XML_FILE_PATH=\\"${CMAKE_CURRENT_SOURCE_DIR}\\")
ADD_DEFINITIONS (-DSOURCE_DIR=\\"${PETAVERSE_SOURCE_DIR}\\")

ADD_SUBDIRECTORY (BuiltIn)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/LearningServerMessages
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/OperationalPetController
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PredicateUpdaters
	${OPENCOG_SOURCE_DIR}/tests/embodiment/Control/OperationalPetController
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/tests/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/tests/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure/BuiltIn
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
        ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
	${OPENCOG_SOURCE_DIR}/tests/embodiment/Control/Procedure
	${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/LearningServerMessages
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
	${OPENCOG_BINARY_DIR}/opencog/embodiment/RuleValidation/VirtualWorldData
	${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
	${OPENCOG_BINARY_DIR}/opencog/util
	${OPENCOG_BINARY_DIR}/opencog/comboreduct
	${OPENCOG_BINARY_DIR}
)

ADD_CXXTEST(RunningComboProcedureUTest)
TARGET_LINK_LIBRARIES(RunningComboProcedureUTest Procedure)
IF(WIN32)
    TARGET_LINK_LIBRARIES(RunningComboProcedureUTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(ComboProcedureRepositoryUTest)
TARGET_LINK_LIBRARIES(ComboProcedureRepositoryUTest Procedure)
IF(WIN32)
    TARGET_LINK_LIBRARIES(ComboProcedureRepositoryUTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(BuiltInProcedureRepositoryUTest)
TARGET_LINK_LIBRARIES(BuiltInProcedureRepositoryUTest Procedure behavior)
IF(WIN32)
    TARGET_LINK_LIBRARIES(BuiltInProcedureRepositoryUTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(RunningBuiltInProcedureUTest)
TARGET_LINK_LIBRARIES(RunningBuiltInProcedureUTest Procedure behavior)
IF(WIN32)
    TARGET_LINK_LIBRARIES(RunningBuiltInProcedureUTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(ProcedureInterpreterUTest)
target_link_libraries(ProcedureInterpreterUTest Procedure behavior)
IF(WIN32)
	TARGET_LINK_LIBRARIES(ProcedureInterpreterUTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(isComboInterpreterUTest)
TARGET_LINK_LIBRARIES(isComboInterpreterUTest Procedure PredicateUpdaters behavior)

ADD_CXXTEST(nearestComboInterpreterUTest)
TARGET_LINK_LIBRARIES(nearestComboInterpreterUTest Procedure PredicateUpdaters behavior)

ADD_CXXTEST(nearestFoodComboInterpreterUTest)
TARGET_LINK_LIBRARIES(nearestFoodComboInterpreterUTest Procedure PredicateUpdaters behavior)

ADD_CXXTEST(randomComboInterpreterUTest)
TARGET_LINK_LIBRARIES(randomComboInterpreterUTest Procedure PredicateUpdaters behavior)

ADD_CXXTEST(existsComboInterpreterUTest)
TARGET_LINK_LIBRARIES(existsComboInterpreterUTest Procedure PredicateUpdaters)

ADD_CXXTEST(scriptsComboInterpreterUTest)
TARGET_LINK_LIBRARIES(scriptsComboInterpreterUTest Procedure PredicateUpdaters behavior)

ADD_CXXTEST(actionsComboInterpreterUTest)
TARGET_LINK_LIBRARIES(actionsComboInterpreterUTest Procedure PredicateUpdaters behavior)

ADD_CXXTEST(ProcedureRepositoryUTest)
TARGET_LINK_LIBRARIES(ProcedureRepositoryUTest Procedure behavior)
