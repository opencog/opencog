OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)
ADD_CUSTOM_TARGET(embodiment_atom_types DEPENDS atom_types.h)

# XXX FIXME TODO Until such time that the learning-server messages are
# removed from the spacemap, we have to build static, to avoid circular
# dependencies. That's just plain wrong.
# ADD_LIBRARY (AtomSpaceExtensions SHARED
ADD_LIBRARY (AtomSpaceExtensions STATIC
    atom_types_init
    #atom_types
    AtomSpaceUtil
    makeVirtualAtom
    CompareAtomTreeTemplate
)

ADD_DEPENDENCIES(AtomSpaceExtensions opencog_atom_types)
ADD_DEPENDENCIES(AtomSpaceExtensions embodiment_atom_types)

TARGET_LINK_LIBRARIES (AtomSpaceExtensions
    util
    atomspace
    query
)
