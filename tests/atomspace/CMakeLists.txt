OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)
ADD_CUSTOM_TARGET(test_custom_atom_types DEPENDS atom_types.h)

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
ADD_CXXTEST(AttentionValueUTest)
ADD_CXXTEST(AtomSpaceUTest)
ADD_CXXTEST(HandleEntryUTest)
ADD_CXXTEST(HandleMapUTest)
ADD_CXXTEST(TemporalUTest)
ADD_CXXTEST(HandleSetUTest)
ADD_CXXTEST(TemporalMapUTest)
ADD_CXXTEST(TemporalTableUTest)
ADD_CXXTEST(TimeServerUTest)
ADD_CXXTEST(SavingLoadingUTest)
ADD_DEPENDENCIES(SavingLoadingUTest test_custom_atom_types) 
ADD_DEPENDENCIES(SavingLoadingUTest opencog_atom_types)
