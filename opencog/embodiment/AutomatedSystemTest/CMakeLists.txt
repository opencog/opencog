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
        server
        util
        ${XERCESC_LIBRARIES}
)

ADD_EXECUTABLE (pbTester PBTesterExecutable)

TARGET_LINK_LIBRARIES (pbTester 
        PBTester
)

