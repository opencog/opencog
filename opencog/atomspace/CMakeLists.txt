ADD_LIBRARY (atomspace SHARED
	Atom.cc
	AtomSpace.cc
	AtomTable.cc
	AttentionValue.cc
	ClassServer.cc
	CompositeTruthValue.cc
	CoreUtils.cc
	HandleEntry.cc
	HandleIterator.cc
	HandleSet.cc
	HandleTemporalPair.cc
	HandleTemporalPairEntry.cc
	HandleToTemporalEntryMap.cc
	IndefiniteTruthValue.cc
	Link.cc
	md5.c
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
	types.cc
	VersionHandle.cc
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/src/util
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
	VersionHandle.h
	DESTINATION "include/opencog/atomspace"
)
