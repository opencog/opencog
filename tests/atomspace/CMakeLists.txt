OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)

LINK_LIBRARIES(
	atomspace
	${Boost_SIGNALS_LIBRARY}
)

# AtomTableUTest requires the XML libraries
IF (EXPAT_FOUND)
	ADD_CXXTEST(AtomTableUTest)
	TARGET_LINK_LIBRARIES(AtomTableUTest xml ${EXPAT_LIBRARIES})
ENDIF (EXPAT_FOUND)


ADD_CXXTEST(ClassServerUTest)
ADD_CXXTEST(AtomUTest)
ADD_CXXTEST(NodeUTest)
ADD_CXXTEST(LinkUTest)
ADD_CXXTEST(TruthValueUTest)
ADD_CXXTEST(SimpleTruthValueUTest)
ADD_CXXTEST(IndefiniteTruthValueUTest)

ADD_CXXTEST(CompositeTruthValueUTest)
TARGET_LINK_LIBRARIES(CompositeTruthValueUTest savable spacetime)

ADD_CXXTEST(AttentionValueUTest)
ADD_CXXTEST(AtomSpaceUTest)
ADD_CXXTEST(AtomSpaceImplUTest)
ADD_CXXTEST(AtomSpaceAsyncUTest)
ADD_CXXTEST(HandleEntryUTest)
ADD_CXXTEST(HandleMapUTest)
ADD_CXXTEST(HandleSetUTest)
ADD_CXXTEST(TLBUTest)
