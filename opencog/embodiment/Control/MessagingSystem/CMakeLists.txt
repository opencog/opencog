CMAKE_POLICY(SET CMP0005 OLD)
#TODO next line
ADD_DEFINITIONS(-D_REENTRANT -DACTION_PLAN_TEST_XML_FILE=\\"${PETAVERSE_SOURCE_DIR}/test/Control/OperationalAvatarController/ActionPlanExample.xml\\")

ADD_LIBRARY (MessagingSystem
	Message.cc
	MessageCentral.cc
	MemoryMessageCentral.cc
	FileMessageCentral.cc
	StringMessage.cc
	TickMessage.cc
    FeedbackMessage.cc
    RawMessage.cc
	RouterMessage.cc
	NetworkElement.cc
	NetworkElementCommon.cc
	MessagingSystemExceptions.cc
	ServerSocket.cc
	Router.cc
	RouterServerSocket.cc
	Spawner.cc
	EmbodimentCogServer
)

IF(COMMAND cmake_policy)
    CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

TARGET_LINK_LIBRARIES (MessagingSystem 
	LearningServerMessages 
	Control 
	util
	pthread 
	xml
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_DATE_TIME_LIBRARY}
	server
	${Boost_SIGNALS_LIBRARY}
        ${Boost_THREAD_LIBRARY}
)

ADD_EXECUTABLE (router RouterExecutable.cc)
TARGET_LINK_LIBRARIES (router 
	MessagingSystem
)

ADD_EXECUTABLE (spawner SpawnerExecutable.cc)
TARGET_LINK_LIBRARIES (spawner 
	MessagingSystem
)

ADD_EXECUTABLE (msgtester MessageTesterExecutable.cc)
TARGET_LINK_LIBRARIES (msgtester
	MessagingSystem
)

#ADD_EXECUTABLE (dummyserver DummyServerExecutable.cc)
#TARGET_LINK_LIBRARIES (dummyserver
	#MessagingSystem
#)
