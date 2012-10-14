
ADD_CUSTOM_TARGET(learing_message_info DEPENDS LearningMessage.info.pb.h)

IF (PROTOBUF_FOUND)
    # Compile proto messages.
    SET(PROTO_ARGS 
        PROTOFILES LearningMessage.info.proto
        OUTPATH ${CMAKE_CURRENT_BUILD_DIR}
    )
    PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_ARGS})
ENDIF (PROTOBUF_FOUND)


ADD_LIBRARY (LearningServerMessages
	LearnMessage
	SchemaMessage
	RewardMessage
	LSCmdMessage
	TrySchemaMessage
	StopLearningMessage
)

TARGET_LINK_LIBRARIES (LearningServerMessages
	MessagingSystem
	AvatarComboVocabulary
	comboreduct
	util
	AtomSpaceExtensions
	atomspace
	space_server
	${PROJECT_LIBRARIES}
)
