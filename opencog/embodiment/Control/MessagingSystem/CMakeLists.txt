
#TODO next line
# XXX 1) the xml formats are obsolete,
# 2) this kind of hard-coded link belongs in the config file
# 3) this is for testing not for production
ADD_DEFINITIONS(-D_REENTRANT -DACTION_PLAN_TEST_XML_FILE=\\"${PETAVERSE_SOURCE_DIR}/test/Control/OperationalAvatarController/ActionPlanExample.xml\\")

ADD_LIBRARY (MessagingSystem SHARED
	Message.cc
	MessageCentral.cc
	MessageFactory.cc
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
	MessageCogServer.cc
)

TARGET_LINK_LIBRARIES (MessagingSystem
	Control
	spacetime
	${Boost_DATE_TIME_LIBRARY}
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_THREAD_LIBRARY}
)

# ------------------------------------------------------

ADD_EXECUTABLE (router RouterExecutable.cc)
TARGET_LINK_LIBRARIES (router 
	MessagingSystem
	lslib
)

IF (HAVE_PROTOBUF)
	TARGET_LINK_LIBRARIES (router LearningServerMessages ${PROTOBUF_LIBRARY})
ENDIF (HAVE_PROTOBUF)

# ------------------------------------------------------

ADD_EXECUTABLE (spawner SpawnerExecutable.cc)
TARGET_LINK_LIBRARIES (spawner 
	MessagingSystem
	lslib
)

IF (HAVE_PROTOBUF)
	TARGET_LINK_LIBRARIES (spawner LearningServerMessages ${PROTOBUF_LIBRARY})
ENDIF (HAVE_PROTOBUF)

# ------------------------------------------------------

ADD_EXECUTABLE (msgtester MessageTesterExecutable.cc)
TARGET_LINK_LIBRARIES (msgtester
	MessagingSystem
	Control
	lslib
)
IF (HAVE_PROTOBUF)
	TARGET_LINK_LIBRARIES (msgtester ${PROTOBUF_LIBRARY})
ENDIF (HAVE_PROTOBUF)


#ADD_EXECUTABLE (dummyserver DummyServerExecutable.cc)
#TARGET_LINK_LIBRARIES (dummyserver
	#MessagingSystem
#)
