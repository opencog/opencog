INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/query
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/util
)

# Runtime types supplied by libnlp-types
LINK_LIBRARIES(
    nlp-types
)

ADD_LIBRARY(question SHARED
	FrameQuery.cc
	SentenceQuery.cc
	WordRelQuery.cc
)

INSTALL (TARGETS question
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	FrameQuery.h
	SentenceQuery.h
	WordRelQuery.h
	DESTINATION "include/opencog/nlp/question"
)
