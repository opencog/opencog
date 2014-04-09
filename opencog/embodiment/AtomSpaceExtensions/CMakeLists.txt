OPENCOG_ADD_ATOM_TYPES(atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	embodiment_types.scm)
ADD_CUSTOM_TARGET(embodiment_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (embodiment-types SHARED
    atom_types_init
)
ADD_DEPENDENCIES(embodiment_atom_types opencog_atom_types)
ADD_DEPENDENCIES(embodiment_atom_types spacetime_atom_types)
ADD_DEPENDENCIES(embodiment_atom_types embodiment_atom_types)

TARGET_LINK_LIBRARIES (embodiment-types
	atomspace
)

ADD_LIBRARY (AtomSpaceExtensions SHARED
    AtomSpaceUtil
    makeVirtualAtom
    CompareAtomTreeTemplate
)

TARGET_LINK_LIBRARIES (AtomSpaceExtensions
	embodiment-types
	query
	spacetime
	atomspace
)
