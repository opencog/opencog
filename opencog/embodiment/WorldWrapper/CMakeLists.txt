INCLUDE_DIRECTORIES (
    ${XERCESC_INCLUDE_DIR}
)

ADD_LIBRARY(WorldWrapper
    WorldWrapper
    WorldWrapperUtil
    WorldWrapperUtilCache
    ShellWorldWrapper
    PAIWorldWrapper
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
    ${XERCESC_LIBRARIES}
)
