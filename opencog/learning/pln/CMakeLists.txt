OPENCOG_ADD_ATOM_TYPES(atom_types.script
                      atom_types.h
                      atom_types.definitions
                      atom_types.inheritance
                      pln_types.scm)
ADD_CUSTOM_TARGET(pln_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (PLNTypes SHARED
	PLNTypes
)

ADD_DEPENDENCIES(PLNTypes opencog_atom_types)
ADD_DEPENDENCIES(PLNTypes pln_atom_types)

TARGET_LINK_LIBRARIES (PLNTypes
	atomspace
)
