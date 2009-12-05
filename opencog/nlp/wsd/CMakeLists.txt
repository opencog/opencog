# ADD_DEFINITIONS(-DDEBUG)

INCLUDE_DIRECTORIES (
    ${ODBC_INCLUDE_DIRS}
)

IF (${ODBC_FOUND})
    LINK_LIBRARIES (wsd persist)
ENDIF (${ODBC_FOUND})

ADD_LIBRARY (wsd SHARED
	EdgeThin.cc
	EdgeUtils.cc
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
	Sweep.cc
	WordSenseProcessor.cc
)

ADD_DEPENDENCIES(wsd nlp_atom_types)

IF (${GUILE_FOUND})
	TARGET_LINK_LIBRARIES(wsd
   	${GUILE_LIBRARIES}
		smob
	)
ENDIF (${GUILE_FOUND})


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
	Sweep.h
	WordSenseProcessor.h
	DESTINATION "include/opencog/nlp/wsd"
)
