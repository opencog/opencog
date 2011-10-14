CMAKE_POLICY(SET CMP0005 OLD)

ADD_DEFINITIONS(-DATOM_SPACE_DUMP_FILE=\\"${CMAKE_CURRENT_SOURCE_DIR}/atomSpace.xml\\")

ADD_CXXTEST(LearnMessageUTest)
TARGET_LINK_LIBRARIES(LearnMessageUTest 
    MessagingSystem
    LearningServerMessages
    util
    Filter
    ${PROJECT_LIBRARIES}
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(LearnMessageUTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(RewardMessageUTest)
TARGET_LINK_LIBRARIES(RewardMessageUTest 
    MessagingSystem
    LearningServerMessages
    SpaceMap
    AvatarComboVocabulary
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
    #util
    #comboreduct
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(SchemaMessageUTest winmm)
ENDIF(WIN32)
