
OPENCOG_ADD_ATOM_TYPES(
	atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
)

ADD_LIBRARY (nlp-types SHARED
	atom_types
	NLPTypes
)

INSTALL (TARGETS nlp-types
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	atom_types.h
	NLPTypes.h
	DESTINATION "include/opencog/nlp/types"
)
