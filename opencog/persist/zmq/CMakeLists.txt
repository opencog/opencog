
IF (HAVE_ZMQ)

ADD_LIBRARY (zmqatoms SHARED
	ProtocolBufferSerializer
	ZMQClient
	ZMQMessages.pb
	ZMQServer
)

TARGET_LINK_LIBRARIES(zmqatoms mq)

ENDIF (HAVE_ZMQ)

