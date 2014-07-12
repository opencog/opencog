OPENCOG_ADD_ATOM_TYPES(
	atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	persist_test_types.scm
)
ADD_CUSTOM_TARGET(test_custom_atom_types DEPENDS atom_types.h)

LINK_LIBRARIES(
	savable
	spacetime
	spacetime-types
	atomspace
)

ADD_CXXTEST(SavingLoadingUTest)
ADD_DEPENDENCIES(SavingLoadingUTest test_custom_atom_types) 
ADD_DEPENDENCIES(SavingLoadingUTest opencog_atom_types)
ADD_DEPENDENCIES(SavingLoadingUTest spacetime_atom_types)
