ADD_LIBRARY(PBTester
	PBTester
	TestConfig
	GoldStdReaderAgent
	GoldStdGen
	GoldStdMessage
)

#	MessagingSystem

TARGET_LINK_LIBRARIES (PBTester 
	PAI
	Control
	server
	space_server
	util
)

ADD_EXECUTABLE (pbTest PBTesterExecutable)

TARGET_LINK_LIBRARIES (pbTest 
	PBTester
	LearningServerMessages
	MessagingSystem
	lslib
	space_server
	server
	SpaceMap
)

IF(HAVE_GUILE)
	TARGET_LINK_LIBRARIES (pbTest smob)
ENDIF(HAVE_GUILE)


