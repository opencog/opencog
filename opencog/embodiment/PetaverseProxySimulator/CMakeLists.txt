ADD_LIBRARY (PVPSimulator
    SimulationConfig
    WorldSimulator
    PhysiologicalModel
    TickerAgent
    InterfaceListenerAgent
    MessageSenderAgent
    PVPSimulator
    AGISimSimulator
)

SET(CMAKE_BUILD_TYPE Debug)

TARGET_LINK_LIBRARIES (PVPSimulator 
    PBTester
    PAI
    MessagingSystem
    AGISimSimProxy
    util
)

IF(COMMAND cmake_policy)
    CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)
ADD_EXECUTABLE (pvpSimulator
	PVPSimulatorExecutable
)
TARGET_LINK_LIBRARIES (pvpSimulator 
        PVPSimulator
)

