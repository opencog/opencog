#SET_DIRECTORY_PROPERTIES(PROPERTIES CLEAN_NO_CUSTOM true)
OPENCOG_ADD_ATOM_TYPES(
	atom_types.script
	atom_types.h
	atom_types.definitions
	atom_types.inheritance
	patternminer_types.scm
	patternminer_types.pyx
)
ADD_CUSTOM_TARGET(patternminer_atom_types DEPENDS atom_types.h)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

ADD_LIBRARY (patternminer-types SHARED
	atom_types
	PatternMinerTypes
)

TARGET_LINK_LIBRARIES(patternminer-types
	${ATOMSPACE_LIBRARIES}
)

# Without this, parallel make will race and crap up the generated files.
ADD_DEPENDENCIES(patternminer-types patternminer_atom_types)

IF (WIN32)
    INSTALL (TARGETS patternminer-types
		DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
    INSTALL (TARGETS patternminer-types
		LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/atom_types.h
	DESTINATION "include/${PROJECT_NAME}/learning/PatternMiner/types"
)
INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/patternminer_types.scm
	DESTINATION "${DATADIR}/scm/opencog/patternminer/"
)
