#INCLUDE_DIRECTORIES (
##${GUILE_INCLUDE_DIRS}
#)

ADD_LIBRARY (ocweb SHARED
    WebModule
    GetAtomRequest
    JsonUtil
    CreateAtomRequest
    UpdateAtomRequest
    GetListRequest
    ServerRequestWrapper
    BaseURLHandler
    AtomURLHandler
    ListURLHandler
    mongoose
    #RequestMapper
    #SimpleAtomSpaceMapper
    #NavigationMapper
)


TARGET_LINK_LIBRARIES(ocweb
    server
    cogutil
    json_spirit
)

INSTALL(FILES
#    resources/init.js
#    resources/processing.js
#    resources/local_graph.js
    index.html
    DESTINATION "${DATADIR}/www/"
)

ADD_SUBDIRECTORY(json_spirit)
ADD_SUBDIRECTORY(resources)

# TODO: take mongoose flags from Makefile for individual platforms. These are
# just for linux:
#SET(MONGOOSE_FLAGS
#    "-W -Wall -std=c99 -pedantic -Os -fomit-frame-pointer -D_POSIX_SOURCE -D_BSD_SOURCE -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -ldl -lpthread"
#)


