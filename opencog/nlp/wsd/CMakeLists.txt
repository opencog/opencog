# ADD_DEFINITIONS(-DDEBUG)
OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)

INCLUDE_DIRECTORIES (
    ${ODBC_INCLUDE_DIRS}
)

IF (${ODBC_FOUND})
    LINK_LIBRARIES (wsd persist)
ENDIF (${ODBC_FOUND})

ADD_LIBRARY (wsd SHARED
	EdgeThin.cc
	EdgeUtils.cc
	atom_types
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
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	EdgeUtils.h
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
