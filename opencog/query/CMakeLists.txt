INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/util
)

# gama's note: I'm adding a dependency on the wsd module because the atom types
# used by the query module are defined there, so it's need at runtime. however,
# I'm unsure if I've placed the definitions in the right place, so linas should
# review this a rearrange as he deems appropriate
LINK_LIBRARIES(
    wsd
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
