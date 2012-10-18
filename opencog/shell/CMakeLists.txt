
ADD_LIBRARY (scheme-shell SHARED
	GenericShell
	SchemeShell
	SchemeShellModule
)

TARGET_LINK_LIBRARIES(scheme-shell
	smob
	server
	atomspace
	util
	${Boost_SYSTEM_LIBRARY}
)

IF (WIN32)
	INSTALL (TARGETS scheme-shell DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
	INSTALL (TARGETS scheme-shell LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
	GenericShell.h
	SchemeShell.h
	SchemeShellModule.h
	DESTINATION "include/${PROJECT_NAME}/shell"
)

