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

ADD_EXECUTABLE (pbTest PBTesterExecutable)

TARGET_LINK_LIBRARIES (pbTest 
        PBTester
)

