
# Always build the atom types, even if not building the rest of
# embodiment.
OPENCOG_ADD_ATOM_TYPES(atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	embodiment_types.scm
	embodiment_types.pyx)
ADD_CUSTOM_TARGET(embodiment_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (embodiment-types SHARED
	atom_types_init
)
ADD_DEPENDENCIES(embodiment-types spacetime_atom_types)
ADD_DEPENDENCIES(embodiment-types embodiment_atom_types)

TARGET_LINK_LIBRARIES (embodiment-types
	${ATOMSPACE_atombase_LIBRARY}
)

INSTALL (FILES
  	${CMAKE_CURRENT_BINARY_DIR}/embodiment_types.scm
  	DESTINATION "${DATADIR}/scm/embodiment"
)

INSTALL (TARGETS embodiment-types
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
