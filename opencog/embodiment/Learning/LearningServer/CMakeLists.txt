
SET(LSLIB_SOURCE_FILES
        LearningServer
	LS
	LSMocky
	SleepAgent
	ImitationLearningAgent
)

ADD_LIBRARY(lslib
${LSLIB_SOURCE_FILES} 
)

SET(lslib_LINK_LIBS
    PetaverseMOSES
    comboreduct
    server
)

IF (HAVE_ZMQ)
  SET(lslib_LINK_LIBS ${lslib_LINK_LIBS} zmq)
ENDIF (HAVE_ZMQ)

TARGET_LINK_LIBRARIES (lslib
    ${lslib_LINK_LIBS}
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
