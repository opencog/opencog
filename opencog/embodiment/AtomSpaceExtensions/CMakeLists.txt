OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)
ADD_CUSTOM_TARGET(embodiment_atom_types DEPENDS atom_types.h)

ADD_LIBRARY (AtomSpaceExtensions
    atom_types
    atom_types_init
    AtomSpaceUtil
    CompareAtomTreeTemplate
)

ADD_DEPENDENCIES(AtomSpaceExtensions opencog_atom_types)

