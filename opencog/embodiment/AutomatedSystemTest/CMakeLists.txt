ADD_LIBRARY(PBTester
	PBTester
	TestConfig
	GoldStdReaderAgent
        GoldStdGen
        GoldStdMessage
)

ADD_EXECUTABLE (pbTester PBTesterExecutable)

TARGET_LINK_LIBRARIES (pbTester 
        PBTester
        PAI
        MessagingSystem
        Control
        server
        util
        ${XERCESC_LIBRARIES}
)

