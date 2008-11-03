SET_DIRECTORY_PROPERTIES(PROPERTIES CLEAN_NO_CUSTOM true)
OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)

ADD_LIBRARY (atomspace SHARED
	Atom.cc
	AtomSpace.cc
	AtomSpaceInit.cc
	AtomTable.cc
	AttentionValue.cc
	ClassServer.cc
	CompositeTruthValue.cc
	CoreUtils.cc
	Handle.cc
	HandleEntry.cc
	HandleIterator.cc
	HandleSet.cc
	HandleTemporalPair.cc
	HandleTemporalPairEntry.cc
	HandleToTemporalEntryMap.cc
	IndefiniteTruthValue.cc
	Link.cc
	Node.cc
	NullTruthValue.cc
	SavingLoading.cc
	SimpleTruthValue.cc
	StatisticsMonitor.cc
	Temporal.cc
	TemporalEntry.cc
	TemporalMap.cc
	TemporalTable.cc
	TemporalToHandleSetMap.cc
	TimeServer.cc
	TLB.cc
	Trail.cc
	TruthValue.cc
	VersionHandle.cc
	atom_types.h
	md5.c
	types.cc
)

TARGET_LINK_LIBRARIES(atomspace
	util
)

INSTALL (TARGETS atomspace
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	Atom.h
	AtomSpaceDefinitions.h
	AtomSpace.h
	AtomTable.h
	atom_types.h
	AttentionValue.h
	ClassServer.h
	CompositeTruthValue.h
	CoreUtils.h
	Defaults.h
	FollowLink.h
	ForeachChaseLink.h
	Foreach.h
	ForeachTwo.h
    Handle.h
	HandleEntry.h
	HandleIterator.h
	HandleMap.h
	HandleSet.h
	HandleTemporalPairEntry.h
	HandleTemporalPair.h
	HandleToTemporalEntryMap.h
	IndefiniteTruthValue.h
	Link.h
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
	VersionHandle.h
	atom_types.h
	md5.h
	types.h
	DESTINATION "include/opencog/atomspace"
)
