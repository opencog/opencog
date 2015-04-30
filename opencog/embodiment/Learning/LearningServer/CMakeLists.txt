
ADD_LIBRARY(lslib
	LearningServer
	LS
	LSMocky
	SleepAgent
	ImitationLearningAgent
)

ADD_LIBRARY(TestPatternMinerAgent SHARED
	TestPatternMinerAgent
)

TARGET_LINK_LIBRARIES (TestPatternMinerAgent
	PatternMiner
)


TARGET_LINK_LIBRARIES (lslib
	PetaverseMOSES
	comboreduct
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
	Filter
	FitnessEstimator
	Procedure
	RewritingRules
	moses
	feature_selection
	moses
	${COGUTIL_LIBRARY}
	comboant
	comboreduct
	pthread
	${Boost_REGEX_LIBRARY}
	${Boost_THREAD_LIBRARY}
)

IF (HAVE_GUILE)
   SET(EXE_LINK_LIBS smob ${EXE_LINK_LIBS})
ENDIF (HAVE_GUILE)

ADD_EXECUTABLE (learningServer LSExecutable)
TARGET_LINK_LIBRARIES(learningServer ${EXE_LINK_LIBS})

ADD_EXECUTABLE (LSMocky LSMockyExec)
TARGET_LINK_LIBRARIES(LSMocky ${EXE_LINK_LIBS})
