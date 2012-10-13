#SET_DIRECTORY_PROPERTIES(PROPERTIES CLEAN_NO_CUSTOM true)
OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)
ADD_CUSTOM_TARGET(opencog_atom_types DEPENDS atom_types.h)

ADD_LIBRARY (atomspace SHARED
    atom_types.h
    Atom.cc
    AtomSpace.cc
    AtomSpaceAsync.cc
    AtomSpaceImpl.cc
    AtomSpaceInit.cc
    AtomTable.cc
    AttentionValue.cc
    AttentionBank.cc
    ClassServer.cc
    CompositeTruthValue.cc
    CountTruthValue.cc
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
    SimpleTruthValue.cc
    StatisticsMonitor.cc
    StringIndex.cc
    TargetTypeIndex.cc
    Temporal.cc
    TemporalEntry.cc
    TemporalMap.cc
    TemporalTable.cc
    TemporalToHandleSetMap.cc
    TLB.cc
    Trail.cc
    TruthValue.cc
    TypeIndex.cc
    VersionHandle.cc
)

ADD_DEPENDENCIES(atomspace opencog_atom_types)

SET(ATOMSPACE_LINK_LIBS
	util
	${GSL_LIBRARIES}
	${Boost_SIGNALS_LIBRARY}
	${Boost_THREAD_LIBRARY}
)

IF (HAVE_ZMQ)
	SET(ATOMSPACE_LINK_LIBS ${ATOMSPACE_LINK_LIBS} zmq)
ENDIF (HAVE_ZMQ)

TARGET_LINK_LIBRARIES(atomspace
	${ATOMSPACE_LINK_LIBS}
)

IF (CYGWIN)
	INSTALL (TARGETS atomspace
		DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
	)
ELSE (CYGWIN)
	INSTALL (TARGETS atomspace
		LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
	)
ENDIF (CYGWIN)

INSTALL (FILES
    ASRequest.h
    Atom.h
    AtomIndex.h
    AtomSpaceDefinitions.h
    AtomSpace.h
    AtomSpaceAsync.h
    AtomSpaceImpl.h
    AtomTable.h
    atom_types.h
    AttentionValue.h
    AttentionBank.h
    BackingStore.h
    ClassServer.h
    CompositeTruthValue.h
    CountTruthValue.h
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
    SimpleTruthValue.h
    SpaceServerContainer.h
    StatisticsMonitor.h
    StringIndex.h
    TargetTypeIndex.h
    TemporalEntry.h
    Temporal.h
    TemporalMap.h
    TemporalTable.h
    TemporalToHandleSetMap.h
    TLB.h
    Trail.h
    TruthValue.h
    TypeIndex.h
    VersionHandle.h
    types.h
    DESTINATION "include/${PROJECT_NAME}/atomspace"
)

ADD_EXECUTABLE (atomspace_async
    atomspace_async_tester
)

TARGET_LINK_LIBRARIES (atomspace_async
    atomspace
    util
)
