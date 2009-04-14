IF(COMMAND cmake_policy)
      CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

ADD_LIBRARY(lslib
        LS
	LSMocky
	SleepAgent
        ImitationLearningAgent
)

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
)
