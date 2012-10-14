
IF (PROTOBUF_FOUND)

	ADD_CUSTOM_TARGET(learing_message_info DEPENDS LearningMessage.info.pb.h)

    # Compile proto messages.
    SET(PROTO_ARGS 
        PROTOFILES LearningMessage.info.proto
        OUTPATH ${CMAKE_CURRENT_BUILD_DIR}
    )
    PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_ARGS})

	SET(MSG_INFO LearningMessage.info.pb.cc)
ELSE (PROTOBUF_FOUND)
	SET(MSG_INFO "")
ENDIF (PROTOBUF_FOUND)


# XXX FIXME TODO Until such time that the learning-server messages are
# removed from the spcemap, we have to build static, to avoid circular
# dependencies. That's just plain wrong.
# ADD_LIBRARY (LearningServerMessages SHARED
ADD_LIBRARY (LearningServerMessages STATIC
	LearnMessage
	SchemaMessage
	RewardMessage
	LSCmdMessage
	TrySchemaMessage
	StopLearningMessage
	${MSG_INFO}
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
