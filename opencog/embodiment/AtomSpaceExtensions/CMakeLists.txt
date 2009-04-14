OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)

ADD_LIBRARY (AtomSpaceExtensions
    atom_types
    atom_types_init
    AtomSpaceUtil
    CompareAtomTreeTemplate
)

ADD_DEPENDENCIES(AtomSpaceExtensions opencog_atom_types)

