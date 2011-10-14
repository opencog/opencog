INCLUDE_DIRECTORIES (
    ${XERCESC_INCLUDE_DIR}
)

ADD_LIBRARY(WorldWrapper
    WorldWrapper
    WorldWrapperUtil
    WorldWrapperUtilCache
    WorldWrapperUtilMock
    ShellWorldWrapper
    PAIWorldWrapper
    RuleValidationWorldWrapper
    NoSpaceLifeWorldWrapper
)

ADD_DEPENDENCIES(WorldWrapper
    PAI
)

TARGET_LINK_LIBRARIES(WorldWrapper
    comboreduct
    util
    PAI
    TangentBug
    AStar
    HPASearch
    MessagingSystem
#    ImaginaryLife
    AvatarComboVocabulary
    VirtualWorldData
    ${XERCESC_LIBRARIES}
)
