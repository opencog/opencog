#SET_DIRECTORY_PROPERTIES(PROPERTIES CLEAN_NO_CUSTOM true)

OPENCOG_ADD_ATOM_TYPES(atom_types.script 
                      atom_types.h
                      atom_types.definitions
                      atom_types.inheritance
                      core_types.scm
                      core_types.pyx)
ADD_CUSTOM_TARGET(opencog_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (atomspace SHARED
    atom_types.h
    Atom.cc
    AtomSpace.cc
    AtomSpaceInit.cc
    AtomTable.cc
    AttentionValue.cc
    AttentionBank.cc
    BackingStore.cc
    ClassServer.cc
    CountTruthValue.cc
    FixedIntegerIndex.cc
    FuzzyTruthValue.cc
    Handle.cc
    HandleSeqIndex.cc
    HandleSetIndex.cc
    ImportanceIndex.cc
    IncomingIndex.cc
    IndefiniteTruthValue.cc
    Link.cc
    LinkIndex.cc
    Node.cc
    NodeIndex.cc
    NullTruthValue.cc
    ProbabilisticTruthValue.cc
    SimpleTruthValue.cc
    TLB.cc
    TruthValue.cc
    TypeIndex.cc
)

# Without this, parallel make will race and crap up the generated files.
ADD_DEPENDENCIES(atomspace opencog_atom_types atom_types.h)

TARGET_LINK_LIBRARIES(atomspace
	lambda
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
    AtomSpace.h
    ${CMAKE_CURRENT_BINARY_DIR}/atom_types.h
    AttentionValue.h
    AttentionBank.h
    BackingStore.h
    ClassServer.h
    CountTruthValue.h
    FollowLink.h
    ForeachChaseLink.h
    Foreach.h
    ForeachTwo.h
    FuzzyTruthValue.h
    Handle.h
    IndefiniteTruthValue.h
    Link.h
    Node.h
    NullTruthValue.h
    ProbabilisticTruthValue.h
    SimpleTruthValue.h
    TLB.h
    TruthValue.h
    types.h
    DESTINATION "include/${PROJECT_NAME}/atomspace"
)
