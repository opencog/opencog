ADD_LIBRARY (atomspace STATIC
	TLB.cc
	Atom.cc         
	CompositeTruthValue.cc  
	HandleEntry.cc     
	IndefiniteTruthValue.cc  
	NullTruthValue.cc     
	Trail.cc       
	utils2.cc
	AtomTable.cc   
	CoreUtils.cc   
	HandleIterator.cc  
	Link.cc   
	SimpleTruthValue.cc 
	TruthValue.cc
	utils.cc
	ClassServer.cc
	HandleMap.cc  
	Node.cc  
	StatisticsMonitor.cc 
	types.cc 
	VersionHandle.cc
	md5.c 	
	Temporal.cc
	AttentionValue.cc
	HandleSet.cc     
	HandleTemporalPairEntry.cc  
	TemporalEntry.cc
	TemporalTable.cc  
	TimeServer.cc
	HandleTemporalPair.cc 
	HandleToTemporalEntryMap.cc
	TemporalMap.cc 
	TemporalToHandleSetMap.cc
	AtomSpace.cc 
	SavingLoading.cc 
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/util
	${OPENCOG_SOURCE_DIR}/src/atomspace
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/src/util
)

TARGET_LINK_LIBRARIES(atomspace
    util
)
