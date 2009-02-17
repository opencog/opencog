IF (WIN32)
	SET(WIN32_GETOPT_FILES getopt getopt_long)
ELSE (WIN32)
	SET(WIN32_GETOPT_FILES)
ENDIF (WIN32)

ADD_LIBRARY(util
	Config
	exceptions
	Logger
	misc
	mt19937ar
	platform
	StringTokenizer
	tree
	${WIN32_GETOPT_FILES}
)

TARGET_LINK_LIBRARIES(util
	${Boost_FILESYSTEM_LIBRARY}
)

INSTALL (FILES
	based_variant.h
	Config.h
	exceptions.h
	hashing.h
	hash_map.h
	hash_set.h
	Logger.h
	misc.h
	mt19937ar.h
	numeric.h
	platform.h
	RandGen.h
	recent_val.h
	StringTokenizer.h
	tree.h
	DESTINATION "include/opencog/util"
)
