INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/LearningServerMessages
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/Filter
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
        ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning/behavior
	${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/LearningServerMessages
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning/Filter
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Learning
	${OPENCOG_BINARY_DIR}/opencog/embodiment/PetComboVocabulary
	${OPENCOG_BINARY_DIR}/opencog/util
)

CMAKE_POLICY(SET CMP0005 OLD)

ADD_DEFINITIONS(-DATOM_SPACE_DUMP_FILE=\\"${CMAKE_CURRENT_SOURCE_DIR}/atomSpace.xml\\")

ADD_CXXTEST(LearnMessageUTest)
TARGET_LINK_LIBRARIES(LearnMessageUTest 
    MessagingSystem
    LearningServerMessages
    util
    Filter
    AtomSpaceExtensions
    ${OPENCOG_LIBRARIES}
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(LearnMessageUTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(RewardMessageUTest)
TARGET_LINK_LIBRARIES(RewardMessageUTest 
	MessagingSystem
	LearningServerMessages
	AtomSpaceExtensions
        SpaceMap
	PetComboVocabulary
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(RewardMessageUTest winmm)
ENDIF(WIN32)

ADD_CXXTEST(SchemaMessageUTest)
TARGET_LINK_LIBRARIES(SchemaMessageUTest 
	MessagingSystem
	LearningServerMessages
	AtomSpaceExtensions
        SpaceMap
	PetComboVocabulary
        util
        comboreduct
)

IF(WIN32)
    TARGET_LINK_LIBRARIES(SchemaMessageUTest winmm)
ENDIF(WIN32)
