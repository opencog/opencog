
OPENCOG_ADD_ATOM_TYPES(atom_types.script 
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	viterbi_types.scm) 

ADD_CUSTOM_TARGET(viterbi_atom_types DEPENDS atom_types.h)


INCLUDE_DIRECTORIES (
	${LINK_GRAMMAR_INCLUDE_DIRS}
)

LINK_LIBRARIES (viterbi)

ADD_LIBRARY (viterbi SHARED
	atom_types_init.cc
	parser.cc
)

ADD_DEPENDENCIES(viterbi opencog_atom_types)
ADD_DEPENDENCIES(viterbi viterbi_atom_types)

TARGET_LINK_LIBRARIES(viterbi
	lg-dict
	query
	atomspace
	${LINK_GRAMMAR_LIBRARIES}
)

IF (WIN32)
	INSTALL (TARGETS viterbi DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSEIF (CYGWIN)
	INSTALL (TARGETS viterbi DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
	INSTALL (TARGETS viterbi LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
	parser.h
	DESTINATION "include/${PROJECT_NAME}/viterbi"
)


