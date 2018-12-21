OPENCOG_ADD_ATOM_TYPES(atom_types.script
   atom_types.h
   atom_types.definitions
   atom_types.inheritance
   octomap_types.scm
   octomap_types.pyx)

ADD_CUSTOM_TARGET(octomap_atom_types DEPENDS atom_types.h)

ADD_LIBRARY (octomap-types SHARED
	atom_types_init.cc
)

TARGET_LINK_LIBRARIES(octomap-types
	${ATOMSPACE_atomcore_LIBRARY}
	${ATOMSPACE_atombase_LIBRARY}
	${ATOMSPACE_atomproto_LIBRARY}
)

ADD_DEPENDENCIES(octomap-types octomap_atom_types)

INSTALL (TARGETS octomap-types
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	octomap.scm
	DESTINATION "${GUILE_SITE_DIR}/opencog"
)

ADD_GUILE_MODULE (FILES
	${CMAKE_CURRENT_BINARY_DIR}/octomap_types.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/spacetime/octomap"
	DEPENDS octomap_atom_types
)


ADD_LIBRARY (atom_octomap SHARED
	AtomOcTree.cc
	AtomOcTreeNode.cc
	OctoMapNode.cc
	OctoMapValue.cc
	TimeOctomap.cc
)


TARGET_LINK_LIBRARIES(atom_octomap
	${OCTOMAP_LIBRARY}
	${OCTOMAP_OCTOMATH_LIBRARY}
	${ATOMSPACE_atomproto_LIBRARY}
	octomap-types
)

INSTALL (TARGETS atom_octomap
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	AtomOcTree.h
	AtomOcTreeNode.h
	TimeOctomap.h
	DESTINATION "include/${PROJECT_NAME}/spacetime/octomap"
)
