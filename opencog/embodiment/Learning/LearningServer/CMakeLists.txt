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

SET(SERVER_LINK_LIBS
    lslib
    PetaverseHC
    MessagingSystem
    LearningServerMessages
    Control
    util
    comboreduct
    pthread
    Filter
    FitnessEstimator
    Procedure
    RewritingRules
)

IF (GUILE_FOUND)
   SET(SERVER_LINK_LIBS smob ${SERVER_LINK_LIBS})
ENDIF (GUILE_FOUND)

ADD_EXECUTABLE (learningServer LSExecutable)
TARGET_LINK_LIBRARIES(learningServer ${SERVER_LINK_LIBS})

ADD_EXECUTABLE (LSMocky LSMockyExec)
TARGET_LINK_LIBRARIES(LSMocky ${SERVER_LINK_LIBS})
