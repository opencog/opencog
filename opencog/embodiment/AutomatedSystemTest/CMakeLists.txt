ADD_LIBRARY(PBTester
	PBTester
	TestConfig
	GoldStdReaderAgent
	GoldStdGen
	GoldStdMessage
)

TARGET_LINK_LIBRARIES (PBTester 
	PAI
	MessagingSystem
	Control
	util
)

ADD_EXECUTABLE (pbTest PBTesterExecutable)

TARGET_LINK_LIBRARIES (pbTest 
	PBTester
	LearningSpace
	LearningServerMessages
	MessagingSystem
	lslib
)

IF(HAVE_GUILE)
	TARGET_LINK_LIBRARIES (pbTest smob)
ENDIF(HAVE_GUILE)


