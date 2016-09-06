#
# If you want to prevent 'make clean' from deleting the files generated
# by this macro (this is especially useful, if you add the generated files
# to the version control repository), add the following command:
# SET_DIRECTORY_PROPERTIES(PROPERTIES CLEAN_NO_CUSTOM true)

OPENCOG_ADD_ATOM_TYPES(atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	example_types.scm
	example_types.pyx)
ADD_CUSTOM_TARGET(examples_atom_types DEPENDS atom_types.h)

# In the CMakeLists.txt for a module (library or executable) that
# depends on the atom types defined by this library, you must add a
# dependency to this target, as follows:
# ADD_DEPENDENCIES(lib_or_exec_name example_atom_types)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (customatomtypes SHARED
	atom_types
	CustomAtomTypesTester
	CustomAtomTypesModule
)

# This module depends on the base opencog atom types.
ADD_DEPENDENCIES(customatomtypes attention_atom_types)
ADD_DEPENDENCIES(customatomtypes examples_atom_types)
