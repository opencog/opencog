
ADD_LIBRARY (atomutils SHARED
	AtomUtils
)

ADD_DEPENDENCIES(atomutils opencog_atom_types)

TARGET_LINK_LIBRARIES(atomutils
	cogutil
)

IF (CYGWIN)
	INSTALL (TARGETS atomutils
		DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
	)
ELSE (CYGWIN)
	INSTALL (TARGETS atomutils
		LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
	)
ENDIF (CYGWIN)

INSTALL (FILES
    AtomUtils.h
    FindUtils.h
    FollowLink.h
    ForeachChaseLink.h
    HandleMap.h
    DESTINATION "include/${PROJECT_NAME}/atomutils"
)
