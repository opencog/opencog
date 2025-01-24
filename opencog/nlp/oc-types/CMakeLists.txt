#SET_DIRECTORY_PROPERTIES(PROPERTIES CLEAN_NO_CUSTOM true)
OPENCOG_ADD_ATOM_TYPES(
	atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	nlp_oc_types.scm
	nlp_oc_types.pyx
)
ADD_CUSTOM_TARGET(nlp_oc_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (nlp-oc-types SHARED
	atom_types.h
	NLPOCTypes.cc
)

TARGET_LINK_LIBRARIES(nlp-oc-types
	${ATOMSPACE_atomtypes_LIBRARY}
)

# Without this, parallel make will race and crap up the generated files.
ADD_DEPENDENCIES(nlp-oc-types nlp_oc_atom_types)

IF (WIN32)
    INSTALL (TARGETS nlp-oc-types
		DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
    INSTALL (TARGETS nlp-oc-types
		LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/atom_types.h
	DESTINATION "include/${PROJECT_NAME}/nlp/oc-types"
)

ADD_GUILE_MODULE (FILES
	${CMAKE_CURRENT_BINARY_DIR}/nlp_oc_types.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/nlp/oc"
	DEPENDS nlp_oc_atom_types
)

ADD_GUILE_EXTENSION(SCM_CONFIG nlp-oc-types "opencog-ext-path-nlp-oc-types")
