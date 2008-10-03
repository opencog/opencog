INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/util
    ${CSOCKETS_INCLUDE_DIRS}
)

ADD_LIBRARY(query SHARED
	FrameQuery.cc
	QueryProcessor.cc
	PatternMatch.cc
	RelexQuery.cc
)

INSTALL (TARGETS query
	LIBRARY DESTINATION "lib/opencog"
)


INSTALL (FILES
	FrameQuery.h
	OutgoingTree.h
	PatternMatch.h
	QueryProcessor.h
	RelexQuery.h
	DESTINATION "include/opencog/query"
)
