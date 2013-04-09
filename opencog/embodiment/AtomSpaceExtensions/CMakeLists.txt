
OPENCOG_ADD_ATOM_TYPES(atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	embodiment_types.scm)
ADD_CUSTOM_TARGET(embodiment_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (AtomSpaceExtensions SHARED
    atom_types_init
    AtomSpaceUtil
    makeVirtualAtom
    CompareAtomTreeTemplate
)

ADD_DEPENDENCIES(AtomSpaceExtensions opencog_atom_types)
ADD_DEPENDENCIES(AtomSpaceExtensions spacetime_atom_types)
ADD_DEPENDENCIES(AtomSpaceExtensions embodiment_atom_types)

TARGET_LINK_LIBRARIES (AtomSpaceExtensions
	query
	spacetime
	atomspace
)
