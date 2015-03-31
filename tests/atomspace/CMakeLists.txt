
LINK_LIBRARIES(
	atomutils
	atomspace
)

IF(HAVE_GUILE)
	ADD_CXXTEST(AtomTableUTest HandleEntry.cc)
	TARGET_LINK_LIBRARIES(AtomTableUTest smob spacetime-types)
ENDIF(HAVE_GUILE)
ADD_DEPENDENCIES(AtomTableUTest spacetime_atom_types)

ADD_CXXTEST(ClassServerUTest)
ADD_CXXTEST(AtomUTest)
ADD_CXXTEST(NodeUTest)
ADD_CXXTEST(LinkUTest)
ADD_CXXTEST(SimpleTruthValueUTest)
ADD_CXXTEST(IndefiniteTruthValueUTest)
ADD_CXXTEST(TVMergeUTest)
ADD_CXXTEST(AttentionValueUTest)
ADD_CXXTEST(TLBUTest)
ADD_CXXTEST(AtomSpaceUTest HandleEntry.cc)
ADD_CXXTEST(AtomSpaceImplUTest HandleEntry.cc)
ADD_CXXTEST(AtomSpaceAsyncUTest)
ADD_CXXTEST(UseCountUTest)
ADD_CXXTEST(MultiSpaceUTest)
ADD_CXXTEST(RemoveUTest)
ADD_CXXTEST(HandleMapUTest)
