ADD_LIBRARY(query STATIC
	FrameQuery.cc
	QueryProcessor.cc
	PatternMatch.cc
	RelexQuery.cc
)

INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/util
)
