IF (WIN32)
	SET(WIN32_GETOPT_FILES getopt getopt_long)
ELSE (WIN32)
	SET(WIN32_GETOPT_FILES)
ENDIF (WIN32)

ADD_LIBRARY(util
	algorithm
	based_variant
	Config
	digraph
	dorepeat
	exceptions
	files
	functional.h
	lazy_normal_selector
	lazy_random_selector
	lazy_selector
	oc_assert
	Logger
	lru_cache
	misc
	misc-test
	mt19937ar
	platform
	StringManipulator
	StringTokenizer
	tree
	${WIN32_GETOPT_FILES}
)

TARGET_LINK_LIBRARIES(util
	${Boost_FILESYSTEM_LIBRARY}
)

INSTALL (FILES
	algorithm.h
	based_variant.h
	Config.h
	digraph.h
	dorepeat.h
	exceptions.h
	files.h
	functional.h
	oc_assert.h
	hashing.h
	hash_map.h
	hash_set.h
	lazy_normal_selector.h
	lazy_random_selector.h
	lazy_selector.h
	Logger.h
	lru_cache.h
	misc.h
	mt19937ar.h
	numeric.h
	platform.h
	RandGen.h
	recent_val.h
	selection.h
	StringManipulator.h
	StringTokenizer.h
	tree.h
	DESTINATION "include/opencog/util"
)
