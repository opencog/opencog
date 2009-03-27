CMAKE_POLICY(SET CMP0005 OLD)
#TODO next line
ADD_DEFINITIONS(-D_REENTRANT -DACTION_PLAN_TEST_XML_FILE=\\"${PETAVERSE_SOURCE_DIR}/test/Control/OperationalPetController/ActionPlanExample.xml\\")

ADD_LIBRARY (MessagingSystem
	Message.cc
	MessageCentral.cc
	MemoryMessageCentral.cc
	#FileMessageCentral.cc
	StringMessage.cc
	TickMessage.cc
	FeedbackMessage.cc
	RouterMessage.cc
	NetworkElement.cc
	MessagingSystemExceptions.cc
	ServerSocket.cc
	Router.cc
	RouterServerSocket.cc
	Spawner.cc
	#DummyServer.cc
	RouterHttpPostSocket.cc
	#MemoryLeakTask.cc
        EmbodimentCogServer
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/LearningServerMessages
	${OPENCOG_SOURCE_DIR}/opencog
	${OPENCOG_SOURCE_DIR}/include
	${OPENCOG_SOURCE_DIR}/include/filesystem
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/LearningServerMessages
	${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Spatial
	${OPENCOG_BINARY_DIR}/opencog/util
	${OPENCOG_BINARY_DIR}/opencog/xml
)

IF(COMMAND cmake_policy)
    CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

TARGET_LINK_LIBRARIES (MessagingSystem 
	LearningServerMessages 
	Control 
	util
	pthread 
	ssl 
	crypto 
	dl 
        xml
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_DATE_TIME_LIBRARY}
        server
        ${CSOCKETS_LIBRARIES}
	${Boost_SIGNALS_LIBRARY}
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
