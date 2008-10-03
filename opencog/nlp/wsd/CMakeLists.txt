# ADD_DEFINITIONS(-DDEBUG)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/util
    ${CSOCKETS_INCLUDE_DIRS}
)

ADD_LIBRARY (wsd SHARED
	Mihalcea.cc
	MihalceaEdge.cc
	MihalceaLabel.cc
	NNAdjust.cc
	ParseRank.cc
	ReportRank.cc
	SenseCache.cc
	SenseRank.cc
	SenseSimilarityLCH.cc
	SenseSimilaritySQL.cc
	WordSenseProcessor.cc
)

INSTALL (TARGETS wsd
	LIBRARY DESTINATION "lib/opencog"
)

INSTALL (FILES
	ForeachWord.h
	MihalceaEdge.h
	Mihalcea.h
	MihalceaLabel.h
	NNAdjust.h
	ParseRank.h
	ReportRank.h
	SenseCache.h
	SenseRank.h
	SenseSimilarity.h
	WordSenseProcessor.h
	DESTINATION "include/opencog/nlp/wsd"
)
