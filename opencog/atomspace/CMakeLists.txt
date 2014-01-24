#SET_DIRECTORY_PROPERTIES(PROPERTIES CLEAN_NO_CUSTOM true)

OPENCOG_ADD_ATOM_TYPES(atom_types.script 
                      atom_types.h
                      atom_types.definitions
                      atom_types.inheritance
                      core_types.scm)
ADD_CUSTOM_TARGET(opencog_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (atomspace SHARED
    atom_types.h
    Atom.cc
    AtomSpace.cc
    AtomSpaceImpl.cc
    AtomSpaceInit.cc
    AtomTable.cc
    AttentionValue.cc
    AttentionBank.cc
    BackingStore.cc
    ClassServer.cc
    CountTruthValue.cc
    FixedIntegerIndex.cc
    Handle.cc
    HandleSeqIndex.cc
    HandleSetIndex.cc
    ImportanceIndex.cc
    IncomingIndex.cc
    IndefiniteTruthValue.cc
    Intersect.cc
    Link.cc
    LinkIndex.cc
    NameIndex.cc
    Node.cc
    NodeIndex.cc
    NullTruthValue.cc
    PredicateIndex.cc
    SimpleTruthValue.cc
    StringIndex.cc
    TargetTypeIndex.cc
    TLB.cc
    TruthValue.cc
    TypeIndex.cc
)

# Without this, parallel make will race and crap up the generated files.
ADD_DEPENDENCIES(atomspace opencog_atom_types)

TARGET_LINK_LIBRARIES(atomspace
	cogutil
	${GSL_LIBRARIES}
	${Boost_THREAD_LIBRARY}
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
    Atom.h
    AtomIndex.h
    AtomSpaceDefinitions.h
    AtomSpace.h
    AtomSpaceImpl.h
    AtomTable.h
    ${CMAKE_CURRENT_BINARY_DIR}/atom_types.h
    AttentionValue.h
    AttentionBank.h
    BackingStore.h
    ClassServer.h
    CountTruthValue.h
    FixedIntegerIndex.h
    FollowLink.h
    ForeachChaseLink.h
    Foreach.h
    ForeachTwo.h
    Handle.h
    HandleMap.h
    HandleSeqIndex.h
    HandleSetIndex.h
    ImportanceIndex.h
    IncomingIndex.h
    IndefiniteTruthValue.h
    Intersect.h
    Link.h
    LinkIndex.h
    NameIndex.h
    Node.h
    NodeIndex.h
    NullTruthValue.h
    PredicateEvaluator.h
    PredicateIndex.h
    SimpleTruthValue.h
    StringIndex.h
    TargetTypeIndex.h
    TLB.h
    TruthValue.h
    TypeIndex.h
    types.h
    DESTINATION "include/${PROJECT_NAME}/atomspace"
)

