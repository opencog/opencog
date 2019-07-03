
# --------------------------------------------------
OPENCOG_ADD_ATOM_TYPES(atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	attention_types.scm
	attention_types.pyx)
ADD_CUSTOM_TARGET(attention_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY(attention-types SHARED
	atom_types_init.cc
)

# Without this, parallel make will race and crap up the generated files.
ADD_DEPENDENCIES(attention-types attention_atom_types)

TARGET_LINK_LIBRARIES(attention-types
	${ATOMSPACE_atomcore_LIBRARY}
	${ATOMSPACE_atombase_LIBRARY}
	${ATOMSPACE_atomproto_LIBRARY}
	${ATOMSPACE_atomtypes_LIBRARY}
)

INSTALL (TARGETS attention-types
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/atom_types.h
	DESTINATION "include/${PROJECT_NAME}/attentionbank/types"
)

# Install the auto-generated atom types as well
ADD_GUILE_MODULE (FILES
	${CMAKE_CURRENT_BINARY_DIR}/attention_types.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/attentionbank/types"
	DEPENDS attention_atom_types
)
ADD_GUILE_EXTENSION(SCM_CONFIG attention-types "opencog-ext-path-attention-types")