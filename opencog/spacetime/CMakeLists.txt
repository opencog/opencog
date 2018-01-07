OPENCOG_ADD_ATOM_TYPES(atom_types.script
   atom_types.h
   atom_types.definitions
   atom_types.inheritance
   spacetime_types.scm
   spacetime_types.pyx)
ADD_CUSTOM_TARGET(spacetime_atom_types DEPENDS atom_types.h)

ADD_LIBRARY (spacetime-types SHARED
	atom_types_init.cc
)

TARGET_LINK_LIBRARIES(spacetime-types
	${ATOMSPACE_atombase_LIBRARY}
)

ADD_DEPENDENCIES(spacetime-types spacetime_atom_types)

INSTALL (TARGETS spacetime-types
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	spacetime.scm
	DESTINATION "${DATADIR}/scm/opencog"
)

INSTALL (FILES
	${CMAKE_CURRENT_BINARY_DIR}/spacetime_types.scm
	DESTINATION "${DATADIR}/scm/opencog/spacetime"
)
