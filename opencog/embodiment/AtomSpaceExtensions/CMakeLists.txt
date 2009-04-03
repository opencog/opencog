OPENCOG_ADD_ATOM_TYPES(atom_types.script atom_types.h atom_types.definitions atom_types.inheritance)

ADD_LIBRARY (AtomSpaceExtensions STATIC 
    atom_types
    atom_types_init
    AtomSpaceUtil
    CompareAtomTreeTemplate
)

ADD_DEPENDENCIES(AtomSpaceExtensions opencog_atom_types)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Spatial
	${OPENCOG_BINARY_DIR}/opencog/util
)

TARGET_LINK_LIBRARIES(AtomSpaceExtensions 
    SpaceMap
    util
)
