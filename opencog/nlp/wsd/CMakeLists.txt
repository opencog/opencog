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

IF (HAVE_GUILE)
	TARGET_LINK_LIBRARIES(wsd
	 	${GUILE_LIBRARIES}
		smob
	)
ENDIF (HAVE_GUILE)


IF (WIN32)
    INSTALL (TARGETS wsd DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
    INSTALL (TARGETS wsd DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
    INSTALL (TARGETS wsd LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

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
	DESTINATION "include/${PROJECT_NAME}/nlp/wsd"
)
