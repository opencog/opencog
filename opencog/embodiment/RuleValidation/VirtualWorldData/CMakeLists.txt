INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/include/xerces
    ${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/lib/linux/xerces
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
    xerces-c
    util
)

