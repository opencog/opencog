

# Pattern miner build file.

ADD_LIBRARY (PatternMiner SHARED
	Pattern
	HTree
	PatternMiner
	PatternMinerBF
	PatternMinerDF
	PatternMinerDistributedWorker
	PatternMinerCentralServer
)

ADD_DEPENDENCIES(PatternMiner embodiment_atom_types)
ADD_DEPENDENCIES(PatternMiner spacetime_atom_types)

TARGET_LINK_LIBRARIES (PatternMiner
	${COGUTIL_LIBRARY}
	${ATOMSPACE_LIBRARIES}
	cpprest.so
)

ADD_LIBRARY(TestPatternMinerAgent SHARED
	TestPatternMinerAgent
)

ADD_LIBRARY(DistributedPatternMinerClient SHARED
	DistributedPatternMinerClient
)

ADD_LIBRARY(DistributedPatternMinerServer SHARED
	DistributedPatternMinerServer
)

TARGET_LINK_LIBRARIES (TestPatternMinerAgent
	PatternMiner
)

TARGET_LINK_LIBRARIES (DistributedPatternMinerClient
	PatternMiner
)

TARGET_LINK_LIBRARIES (DistributedPatternMinerServer
	PatternMiner
)

INSTALL (FILES
	Pattern.h
	HTree.h
	PatternMiner.h

	DESTINATION "include/${PROJECT_NAME}/PatternMiner"
)
