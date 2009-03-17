CMAKE_POLICY(SET CMP0005 OLD)
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
	DummyServer.cc
	RouterHttpPostSocket.cc
	#MemoryLeakTask.cc
)

INCLUDE_DIRECTORIES (
	${PETAVERSE_SOURCE_DIR}/src/Spatial
	${PETAVERSE_SOURCE_DIR}/src/Spatial/Math
	${PETAVERSE_SOURCE_DIR}/src/AtomSpaceExtensions
	${PETAVERSE_SOURCE_DIR}/src/Control
	${PETAVERSE_SOURCE_DIR}/src/Control/MessagingSystem
	${PETAVERSE_SOURCE_DIR}/src/Learning
	${PETAVERSE_SOURCE_DIR}/src/Learning/LearningServerMessages
	${PETAVERSE_SOURCE_DIR}/include
	${PETAVERSE_SOURCE_DIR}/include/filesystem
)

LINK_DIRECTORIES (
	${PETAVERSE_BINARY_DIR}/src/Control/MessagingSystem
	${PETAVERSE_BINARY_DIR}/src/Learning/LearningServerMessages
	${PETAVERSE_BINARY_DIR}/src/AtomSpaceExtensions
	${PETAVERSE_BINARY_DIR}/src/Spatial
)

IF(COMMAND cmake_policy)
    CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

TARGET_LINK_LIBRARIES (MessagingSystem 
	LearningServerMessages 
	Control 
	${LADSUTIL_LIBRARY} 
	pthread 
	ssl 
	crypto 
	dl 
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_DATE_TIME_LIBRARY}
    ${OPENCOG_LIBRARIES}
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

ADD_EXECUTABLE (dummyserver DummyServerExecutable.cc)
TARGET_LINK_LIBRARIES (dummyserver
	MessagingSystem
)
