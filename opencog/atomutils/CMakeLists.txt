
ADD_LIBRARY (atomutils SHARED
    AtomSpaceUtils.cc
)

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
    AtomSpaceUtils.h
    FollowLink.h
    ForeachChaseLink.h
    Foreach.h
    ForeachTwo.h
    DESTINATION "include/${PROJECT_NAME}/atomutils"
)
