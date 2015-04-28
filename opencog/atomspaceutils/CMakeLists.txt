
ADD_LIBRARY (atomspaceutils SHARED
	AtomSpaceUtils
)

ADD_DEPENDENCIES(atomspaceutils opencog_atom_types)

# TARGET_LINK_LIBRARIES(atomspaceutils
# 	atomspace
# 	${COGUTIL_LIBRARY}
# )

#IF (CYGWIN)
#	INSTALL (TARGETS atomspaceutils
#		DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
#	)
#ELSE (CYGWIN)
#	INSTALL (TARGETS atomspaceutils
#		LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
#	)
#ENDIF (CYGWIN)

INSTALL (FILES
    AtomSpaceUtils.h
    DESTINATION "include/${PROJECT_NAME}/atomutils"
)
