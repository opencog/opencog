IF (WIN32 AND NOT UNIX)
	SET(WIN32_GETOPT_FILES getopt getopt_long)
ELSE (WIN32 AND NOT UNIX)
	SET(WIN32_GETOPT_FILES)
ENDIF (WIN32 AND NOT UNIX)

IF (APPLE)
    SET(APPLE_STRNDUP_FILES strndup strnlen)
ELSE (APPLE)
    SET(APPLE_STRNDUP_FILES)
ENDIF (APPLE)

ADD_LIBRARY(util SHARED
    ansi
	algorithm
    log_prog_name
	based_variant
	cluster
	Config
	Cover_Tree
	digraph
	dorepeat
	exceptions
	files
	functional.h
    iostreamContainer
	lazy_normal_selector
	lazy_random_selector
	lazy_selector
	oc_assert
	Logger
	lru_cache
	misc
	mt19937ar
    random
	platform
	StringManipulator
	StringTokenizer
    octime
	tree
    oc_omp
	${WIN32_GETOPT_FILES}
    ${APPLE_STRNDUP_FILES}
)

TARGET_LINK_LIBRARIES(util
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	${Boost_REGEX_LIBRARY}
	${Boost_THREAD_LIBRARY}
)

INSTALL(FILES
    ansi.h
	algorithm.h
    log_prog_name.h
	based_variant.h
	cluster.h
	Config.h
	Cover_Tree.h
    concurrent_queue.h
	digraph.h
	dorepeat.h
	exceptions.h
	files.h
    foreach.h
	functional.h
	oc_assert.h
	hashing.h
    iostreamContainer.h
	lazy_normal_selector.h
	lazy_random_selector.h
	lazy_selector.h
	Logger.h
	lru_cache.h
	misc.h
	mt19937ar.h
    random.h
	numeric.h
	platform.h
	RandGen.h
	recent_val.h
	selection.h
	StringManipulator.h
	StringTokenizer.h
	tree.h
    octime.h
    oc_omp.h
	DESTINATION "include/${PROJECT_NAME}/util"
)

# is it useful to install since it is static ?
IF (WIN32)
   INSTALL(TARGETS util DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
   INSTALL(TARGETS util LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)
