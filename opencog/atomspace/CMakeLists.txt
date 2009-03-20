SET_DIRECTORY_PROPERTIES(PROPERTIES CLEAN_NO_CUSTOM true)
OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)
ADD_CUSTOM_TARGET(opencog_atom_types DEPENDS atom_types.h)

ADD_LIBRARY (atomspace SHARED
	atom_types.h
	Atom.cc
	AtomSpace.cc
	AtomSpaceInit.cc
	AtomTable.cc
	AttentionValue.cc
	ClassServer.cc
	CompositeTruthValue.cc
	CountTruthValue.cc
	CoreUtils.cc
	FixedIntegerIndex.cc
	Handle.cc
	HandleEntry.cc
	HandleIterator.cc
	HandleSeqIndex.cc
	HandleSet.cc
	HandleTemporalPair.cc
	HandleTemporalPairEntry.cc
	HandleToTemporalEntryMap.cc
	ImportanceIndex.cc
	IndefiniteTruthValue.cc
	Link.cc
	LinkIndex.cc
	NameIndex.cc
	Node.cc
	NodeIndex.cc
	NullTruthValue.cc
	PredicateIndex.cc
	SavingLoading.cc
	SimpleTruthValue.cc
	StatisticsMonitor.cc
	StringIndex.cc
	TargetTypeIndex.cc
	Temporal.cc
	TemporalEntry.cc
	TemporalMap.cc
	TemporalTable.cc
	TemporalToHandleSetMap.cc
	TimeServer.cc
	TLB.cc
	Trail.cc
	TruthValue.cc
	TypeIndex.cc
	utils.cc
#	utils2.cc
	VersionHandle.cc
	md5.c
	types.cc
)

TARGET_LINK_LIBRARIES(atomspace
	util
	${GSL_LIBRARIES}
	${Boost_SIGNALS_LIBRARY}
)

INSTALL (TARGETS atomspace
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	Atom.h
	AtomIndex.h
	AtomSpaceDefinitions.h
	AtomSpace.h
	AtomTable.h
	atom_types.h
	AttentionValue.h
	BackingStore.h
	ClassServer.h
	CompositeTruthValue.h
	CountTruthValue.h
	CoreUtils.h
	Defaults.h
	FixedIntegerIndex.h
	FollowLink.h
	ForeachChaseLink.h
	Foreach.h
	ForeachTwo.h
	Handle.h
	HandleEntry.h
	HandleIterator.h
	HandleMap.h
	HandleSeqIndex.h
	HandleSet.h
	HandleTemporalPairEntry.h
	HandleTemporalPair.h
	HandleToTemporalEntryMap.h
	IndefiniteTruthValue.h
	ImportanceIndex.h
	Link.h
	LinkIndex.h
	NameIndex.h
	Node.h
	NodeIndex.h
	NullTruthValue.h
	PredicateEvaluator.h
	PredicateIndex.h
	SavableRepository.h
	SavingLoading.h
	SimpleTruthValue.h
	StatisticsMonitor.h
	StringIndex.h
	TargetTypeIndex.h
	TemporalEntry.h
	Temporal.h
	TemporalMap.h
	TemporalTable.h
	TemporalToHandleSetMap.h
	TimeServer.h
	TLB.h
	Trail.h
	TruthValue.h
	TypeIndex.h
	VersionHandle.h
	md5.h
	types.h
	utils.h
#	utils2.h
	DESTINATION "include/opencog/atomspace"
)
