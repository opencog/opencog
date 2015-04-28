
ADD_DEFINITIONS(-DATOM_SPACE_DUMP_FILE=\\"${CMAKE_CURRENT_SOURCE_DIR}/atomSpace.xml\\")

# At this time, the LearnMessageUTest test does not compile,
# because of the 2D spacemap ->3D transition.
IF(0)
ADD_CXXTEST(LearnMessageUTest)
TARGET_LINK_LIBRARIES(LearnMessageUTest 
	MessagingSystem
	LearningServerMessages
	cogutil
	Filter
	${PROJECT_LIBRARIES}
)

IF(WIN32)
	TARGET_LINK_LIBRARIES(LearnMessageUTest winmm)
ENDIF(WIN32)
ENDIF(0)

ADD_CXXTEST(RewardMessageUTest)
TARGET_LINK_LIBRARIES(RewardMessageUTest 
	MessagingSystem
	LearningServerMessages
	Control
	SpaceMap
	AvatarComboVocabulary
	server
)

IF(WIN32)
	TARGET_LINK_LIBRARIES(RewardMessageUTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(SchemaMessageUTest)
TARGET_LINK_LIBRARIES(SchemaMessageUTest 
	MessagingSystem
	LearningServerMessages
	SpaceMap
	AvatarComboVocabulary
	server
)

IF(WIN32)
	TARGET_LINK_LIBRARIES(SchemaMessageUTest winmm)
ENDIF(WIN32)
