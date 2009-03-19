INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog
    ${XERCESC_INCLUDE_DIR}
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/util
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

