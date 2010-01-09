#INCLUDE_DIRECTORIES (
##${GUILE_INCLUDE_DIRS}
#)

ADD_LIBRARY (ocweb SHARED
    WebModule
    GetAtomRequest
    GetListRequest
    ServerRequestWrapper
    #    AddAtomRequest
    mongoose
    #RequestMapper
    #SimpleAtomSpaceMapper
    #NavigationMapper
)

TARGET_LINK_LIBRARIES(ocweb
    server
)

INSTALL(FILES
    init.js
    processing.js
    local_graph.js
    DESTINATION "${DATADIR}/www"
)

# TODO: take mongoose flags from Makefile for individual platforms. These are
# just for linux:
#SET(MONGOOSE_FLAGS
#    "-W -Wall -std=c99 -pedantic -Os -fomit-frame-pointer -D_POSIX_SOURCE -D_BSD_SOURCE -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -ldl -lpthread"
#)


