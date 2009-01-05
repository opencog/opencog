INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/util
)

ADD_LIBRARY(query SHARED
	FrameQuery.cc
	QueryProcessor.cc
	PatternMatch.cc
	PatternMatchEngine.cc
	RelexQuery.cc
)

INSTALL (TARGETS query
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	DefaultPatternMatchCB.h
	FrameQuery.h
	OutgoingTree.h
	PatternMatch.h
	PatternMatchCallback.h
	PatternMatchEngine.h
	QueryProcessor.h
	RelexQuery.h
	DESTINATION "include/opencog/query"
)
