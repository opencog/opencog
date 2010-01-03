#INCLUDE_DIRECTORIES (
##${GUILE_INCLUDE_DIRS}
#)

ADD_LIBRARY (rest SHARED
    RESTModule
    GetAtomRequest
    GetListRequest
    #    AddAtomRequest
    mongoose
    #RequestMapper
    #SimpleAtomSpaceMapper
    #NavigationMapper
)

TARGET_LINK_LIBRARIES(rest
    server
)

# TODO: take mongoose flags from Makefile for individual platforms. These are
# just for linux:
#SET(MONGOOSE_FLAGS
#    "-W -Wall -std=c99 -pedantic -Os -fomit-frame-pointer -D_POSIX_SOURCE -D_BSD_SOURCE -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -ldl -lpthread"
#)


