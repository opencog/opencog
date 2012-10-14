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
	${XERCESC_LIBRARIES}
)

ADD_EXECUTABLE (pbTest PBTesterExecutable)

TARGET_LINK_LIBRARIES (pbTest 
	PBTester
)

