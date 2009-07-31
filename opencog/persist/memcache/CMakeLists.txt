# Uncomment this if libmemcahced is installed.
# ADD_DEFINITIONS(-DHAVE_LIBMEMCACHED)

#SET(LIBMEMCACHED_INCLUDE_DIRS, /usr/local/include/libmemcached)

INCLUDE_DIRECTORIES (
	${LIBMEMCACHED_INCLUDE_DIRS}
)

ADD_LIBRARY(cachedb STATIC
	AtomCache.cc
)

ADD_EXECUTABLE(sniff
	sniff.cc
)

ADD_EXECUTABLE(sniff2
	sniff2.cc
)

TARGET_LINK_LIBRARIES(cachedb memcached)

TARGET_LINK_LIBRARIES(sniff
	atomspace
	${LIBMEMCACHED_LIBRARIES}
)

TARGET_LINK_LIBRARIES(sniff2
	cachedb
	atomspace
	${LIBMEMCACHED_LIBRARIES}
)
