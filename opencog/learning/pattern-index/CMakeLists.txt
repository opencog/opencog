ADD_LIBRARY (patternindex SHARED
	PatternIndexAPI
	PatternIndexSCM
	TypeFrameIndex
	TypeFrameIndexBuilder
	PatternHeap
	CombinationGenerator
	PartitionGenerator
	CartesianProductGenerator
	SCMLoader
	TypeFrame
)

ADD_DEPENDENCIES (patternindex
	nlp_atom_types			
)

TARGET_LINK_LIBRARIES(patternindex
	${ATOMSPACE_LIBRARIES}
)

ADD_EXECUTABLE (patternIndexQueryExample patternIndexQueryExample.cc)
ADD_EXECUTABLE (patternIndexMiningExample patternIndexMiningExample.cc)
TARGET_LINK_LIBRARIES(patternIndexQueryExample
	patternindex
	${ATOMSPACE_LIBRARIES}
	${COGUTIL_LIBRARY}
)
TARGET_LINK_LIBRARIES(patternIndexMiningExample
	patternindex
	${ATOMSPACE_LIBRARIES}
	${COGUTIL_LIBRARY}
)

INSTALL (FILES
	SCMLoader.h
	SCMLoaderCallback.h
	CombinationGenerator.h
	PartitionGenerator.h
	CartesianProductGenerator.h
	PatternHeap.h
	PatternIndexAPI.h
	PatternIndexSCM.h
	TypeFrame.h
	TypeFrameIndex.h
	TypeFrameIndexBuilder.h
	DESTINATION "include/${PROJECT_NAME}/learning/pattern-index"
)

INSTALL (TARGETS patternindex DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_GUILE_MODULE (FILES
	pattern-index.scm
	MODULE_DESTINATION "${DATADIR}/scm/opencog/learning/pattern-index"
)
