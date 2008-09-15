ADD_LIBRARY (atomspace SHARED
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

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/src/util
)

TARGET_LINK_LIBRARIES(atomspace
	util
)

INSTALL (TARGETS atomspace
	LIBRARY DESTINATION "lib/opencog"
)

INSTALL (FILES
	Atom.h
	AtomSpaceDefinitions.h
	AtomSpace.h
	AtomTable.h
	AttentionValue.h
	classes.h
	ClassServer.h
	CompositeTruthValue.h
	CoreUtils.h
	Defaults.h
	FollowLink.h
	ForeachChaseLink.h
	Foreach.h
	ForeachTwo.h
	HandleEntry.h
	HandleIterator.h
	HandleMap.h
	HandleSet.h
	HandleTemporalPairEntry.h
	HandleTemporalPair.h
	HandleToTemporalEntryMap.h
	IndefiniteTruthValue.h
	Link.h
	md5.h
	Node.h
	NullTruthValue.h
	PredicateEvaluator.h
	SavableRepository.h
	SavingLoading.h
	SimpleTruthValue.h
	StatisticsMonitor.h
	TemporalEntry.h
	Temporal.h
	TemporalMap.h
	TemporalTable.h
	TemporalToHandleSetMap.h
	TimeServer.h
	TLB.h
	Trail.h
	TruthValue.h
    type_classint.h
    type_codes.h
    type_inheritance.h
    type_intclass.h
    type_names.h
	types.h
	utils2.h
	utils.h
	VersionHandle.h
	DESTINATION "include/opencog/atomspace"
)
