ADD_LIBRARY(lslib
	LS
        LS_CogServer
	LSMocky
	SleepTask
	ImitationLearningTask
)

INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/PetaverseMOSES
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/RewritingRules
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/LearningServerMessages
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/PetaverseHC
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/FitnessEstimator
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/Filter
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/ImitationLearningAPI
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Procedure/BuiltIn
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_SOURCE_DIR}/opencog
    ${OPENCOG_SOURCE_DIR}/opencog/learning/hillclimbing
    ${OPENCOG_SOURCE_DIR}/opencog/learning/moses
)

LINK_DIRECTORIES(
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/PerceptionActionInterface
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/PetaverseHC
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/behavior
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/FitnessEstimator
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/LearningServerMessages
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/Filter
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/PetaverseMOSES
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/RewritingRules
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/Procedure
    ${OPENCOG_BINARY_DIR}/opencog/util
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/learning/hillclimbing
    ${OPENCOG_BINARY_DIR}/opencog/learning/moses
    ${OPENCOG_BINARY_DIR}/opencog/server
)

IF(COMMAND cmake_policy)
      CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

TARGET_LINK_LIBRARIES (lslib
    PetaverseMOSES
    comboreduct
    server
)

ADD_EXECUTABLE (learningServer LSExecutable)
TARGET_LINK_LIBRARIES (learningServer
    lslib
    PetaverseHC
    MessagingSystem
    LearningServerMessages
    Control
    util
    comboreduct
    pthread
    ssl
    crypto
    dl
    Filter
    FitnessEstimator
    Procedure
    moses
    RewritingRules
    ${OPENCOG_LIBRARIES}
)

ADD_EXECUTABLE (learningServer_CogServer LS_CogServerExecutable)
TARGET_LINK_LIBRARIES (learningServer_CogServer
    lslib
    PetaverseHC
    MessagingSystem
    LearningServerMessages
    Control
    util
    comboreduct
    pthread
    ssl
    crypto
    dl
    Filter
    FitnessEstimator
    Procedure
    moses
    RewritingRules
    ${OPENCOG_LIBRARIES}
)

ADD_EXECUTABLE (LSMocky LSMockyExec)
TARGET_LINK_LIBRARIES (LSMocky
    lslib
    PetaverseHC
    MessagingSystem
    LearningServerMessages
    Control
    util
    comboreduct
    PetComboVocabulary
    pthread
    ssl
    crypto
    dl
    behavior
    Procedure
    moses
    RewritingRules
    ${OPENCOG_LIBRARIES}
)
