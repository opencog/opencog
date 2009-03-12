OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)
ADD_LIBRARY (customatomtypes SHARED
	atom_types
	CustomAtomTypesTester
	CustomAtomTypesModule
)
