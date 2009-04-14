INCLUDE_DIRECTORIES (
    ${XERCESC_INCLUDE_DIR}
)

ADD_LIBRARY(VirtualWorldData
    VirtualWorldState
    VirtualEntity
    VirtualWorldActions
    XmlLoader
    XmlErrorHandler
)

TARGET_LINK_LIBRARIES(VirtualWorldData
    ${XERCESC_LIBRARIES}
    util
)

