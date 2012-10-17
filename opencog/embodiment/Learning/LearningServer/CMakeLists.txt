
ADD_LIBRARY(lslib
	LearningServer
	LS
	LSMocky
	SleepAgent
	ImitationLearningAgent
)

TARGET_LINK_LIBRARIES (lslib
	PetaverseMOSES
	comboreduct
	SpaceMap
	space_server
)

IF (HAVE_ZMQ)
	TARGET_LINK_LIBRARIES (lslib ${lslib_LINK_LIBS} zmq)
ENDIF (HAVE_ZMQ)


SET(EXE_LINK_LIBS
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
	# Procedure
	RewritingRules
)

IF (GUILE_FOUND)
   SET(EXE_LINK_LIBS smob ${EXE_LINK_LIBS})
ENDIF (GUILE_FOUND)

ADD_EXECUTABLE (learningServer LSExecutable)
TARGET_LINK_LIBRARIES(learningServer ${EXE_LINK_LIBS})

ADD_EXECUTABLE (LSMocky LSMockyExec)
TARGET_LINK_LIBRARIES(LSMocky ${EXE_LINK_LIBS})
