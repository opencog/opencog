IF (WIN32)
	SET(WIN32_GETOPT_FILES getopt getopt_long)
ELSE (WIN32)
	SET(WIN32_GETOPT_FILES)
ENDIF (WIN32)

ADD_LIBRARY(util
	Config
	exceptions
	Logger
	mt19937ar
	platform.cc
	StringTokenizer
	tree
	${WIN32_GETOPT_FILES}
)

INSTALL (FILES
	based_variant.h
	Config.h
	exceptions.h
	hashing.h
	hash_map.h
	hash_set.h
	lazy_random_selector.h
	Logger.h
	mt19937ar.h
	numeric.h
	platform.h
	RandGen.h
	recent_val.h
	StringTokenizer.h
	tree.h
	DESTINATION "include/opencog/util"
)
