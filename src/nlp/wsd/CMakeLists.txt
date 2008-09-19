ADD_LIBRARY (wsd STATIC
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

# ADD_DEFINITIONS(-DDEBUG)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/persist
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/util
	${ODBC_INCLUDE_DIRS}
)
