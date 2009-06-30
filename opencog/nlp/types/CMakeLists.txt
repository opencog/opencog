#SET_DIRECTORY_PROPERTIES(PROPERTIES CLEAN_NO_CUSTOM true)
OPENCOG_ADD_ATOM_TYPES(
	atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
)
ADD_CUSTOM_TARGET(nlp_atom_types DEPENDS atom_types.h)

ADD_LIBRARY (nlp-types SHARED
	atom_types
	NLPTypes
)

INSTALL (TARGETS nlp-types
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	atom_types.h
#	NLPTypes.h
	DESTINATION "include/opencog/nlp/types"
)
