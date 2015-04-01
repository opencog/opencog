OPENCOG_ADD_ATOM_TYPES(atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	atom_test_types.scm)

LINK_LIBRARIES(AtomUtilsUTest
	atomutils
	atomspace
)

ADD_CXXTEST(AtomUtilsUTest)
