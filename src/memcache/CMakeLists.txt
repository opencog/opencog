# Uncomment this if libmemcahced is installed.
ADD_DEFINITIONS(-DHAVE_LIBMEMCACHED)

#SET(LIBMEMCACHED_INCLUDE_DIRS, /usr/local/include/libmemcached)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/AtomSpace
	${OPENCOG_SOURCE_DIR}/src/Util
#	${LIBMEMCACHED_INCLUDE_DIRS}
	/usr/local/include/libmemcached
)

ADD_LIBRARY(cachedb STATIC
	AtomCache.cc
)

ADD_EXECUTABLE(sniff
	sniff.cc
)

TARGET_LINK_LIBRARIES(cachedb memcached)

TARGET_LINK_LIBRARIES(sniff
	AtomSpace
	${LIBMEMCACHED_LIBRARIES}
	memcached
)
