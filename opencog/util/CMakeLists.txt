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

# The below are used to automatically find module load paths
ADD_DEFINITIONS(
   -DCMAKE_INSTALL_PREFIX="\\"${CMAKE_INSTALL_PREFIX}\\""
)

ADD_LIBRARY(cogutil SHARED
	ansi
	algorithm
	backtrace-symbols
	based_variant
	cluster
	comprehension
	Config
	Cover_Tree
	digraph
	dorepeat
	exceptions
	files
	functional.h
	iostreamContainer
	KLD
	lazy_normal_selector
	lazy_random_selector
	lazy_selector
	log_prog_name
	Logger
	lru_cache
	MannWhitneyU
	misc
	mt19937ar
	oc_assert
	oc_omp
	octime
	platform
	random
	ranking
	StringManipulator
	StringTokenizer
	tree
	${WIN32_GETOPT_FILES}
	${APPLE_STRNDUP_FILES}
)

TARGET_LINK_LIBRARIES(cogutil
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	${Boost_REGEX_LIBRARY}
	${Boost_THREAD_LIBRARY}
)

IF (HAVE_BFD AND HAVE_IBERTY)
	INCLUDE_DIRECTORIES(${IBERTY_INCLUDE_DIR})
	TARGET_LINK_LIBRARIES(cogutil
		${BFD_LIBRARY}
		iberty
	)
ENDIF (HAVE_BFD AND HAVE_IBERTY)

INSTALL(FILES
	ansi.h
	algorithm.h
	backtrace-symbols.h
	based_variant.h
	cluster.h
	comprehension.h
	Config.h
	Counter.h
	Cover_Tree.h
	concurrent_queue.h
	concurrent_stack.h
	digraph.h
	dorepeat.h
	exceptions.h
	files.h
	foreach.h
	functional.h
	hashing.h
	iostreamContainer.h
	KLD.h
	lazy_normal_selector.h
	lazy_random_selector.h
	lazy_selector.h
	log_prog_name.h
	Logger.h
	lru_cache.h
	macros.h
	MannWhitneyU.h
	misc.h
	mt19937ar.h
	numeric.h
    jaccard_index.h
	oc_assert.h
	oc_omp.h
	octime.h
	platform.h
	pool.h
	RandGen.h
	random.h
	ranking.h
	recent_val.h
	selection.h
	StringManipulator.h
	StringTokenizer.h
	tree.h
	DESTINATION "include/${PROJECT_NAME}/util"
)

IF (WIN32)
   INSTALL(TARGETS cogutil DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
   INSTALL(TARGETS cogutil DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
   INSTALL(TARGETS cogutil LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)
