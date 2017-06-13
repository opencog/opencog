ADD_LIBRARY (scmloader SHARED
	SCMLoader
	PGStorageDelegate
	TypeFrameIndexBuilder
	TypeFrameIndex
    PatternHeap
    PatternIndexAPI
    CombinationGenerator
    PartitionGenerator
    CartesianProductGenerator
	TypeFrame
)

ADD_DEPENDENCIES (scmloader
	nlp_atom_types            
)

TARGET_LINK_LIBRARIES(scmloader
	${ATOMSPACE_LIBRARIES}
)


ADD_EXECUTABLE (loadscm loadscm.cc)
ADD_EXECUTABLE (frame frame.cc)
ADD_EXECUTABLE (scm2pgsql scm2pgsql.cc)
ADD_EXECUTABLE (patternIndexQueryExample patternIndexQueryExample.cc)
ADD_EXECUTABLE (patternIndexMiningExample patternIndexMiningExample.cc)
TARGET_LINK_LIBRARIES(loadscm
	scmloader
)
TARGET_LINK_LIBRARIES(frame
	scmloader
)
TARGET_LINK_LIBRARIES(scm2pgsql
	scmloader
)
TARGET_LINK_LIBRARIES(patternIndexQueryExample
	scmloader
)
TARGET_LINK_LIBRARIES(patternIndexMiningExample
	scmloader
)

INSTALL (TARGETS scmloader DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
INSTALL (TARGETS scm2pgsql RUNTIME DESTINATION "bin")
INSTALL (TARGETS patternIndexQueryExample RUNTIME DESTINATION "bin")
INSTALL (TARGETS patternIndexMiningExample RUNTIME DESTINATION "bin")
INSTALL (FILES
	SCMLoader.h
	PGStorageDelegate.h
	SCMLoaderCallback.h
    CombinationGenerator.h
    PartitionGenerator.h
    CartesianProductGenerator.h
    PatternHeap.h
    PatternIndexAPI.h
	TypeFrame.h
	TypeFrameIndex.h
	TypeFrameIndexBuilder.h
	DESTINATION "include/${PROJECT_NAME}/nlp/scm2pgsql"
)
