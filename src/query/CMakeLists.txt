ADD_LIBRARY(query STATIC
	FrameQuery.cc
	QueryProcessor.cc
	PatternMatch.cc
	RelexQuery.cc
)

INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/AtomSpace
	${OPENCOG_SOURCE_DIR}/src/CogServer
	${OPENCOG_SOURCE_DIR}/src/Util
)
