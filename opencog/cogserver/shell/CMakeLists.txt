INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
)

IF (HAVE_CYTHON)
	INCLUDE_DIRECTORIES(
		${PYTHON_INCLUDE_PATH}
	)

	ADD_LIBRARY (py-shell SHARED
		GenericShell
		PythonShell
		PythonShellModule
	)

	TARGET_LINK_LIBRARIES(py-shell
		server
		${ATOMSPACE_smob_LIBRARY}
		${ATOMSPACE_LIBRARY}
		${COGUTIL_LIBRARY}
		${Boost_SYSTEM_LIBRARY}
		${PYTHON_LIBRARIES}
	)
ENDIF (HAVE_CYTHON)

ADD_LIBRARY (scheme-shell SHARED
	GenericShell
	SchemeShell
	SchemeShellModule
)

TARGET_LINK_LIBRARIES(scheme-shell
	server # setting the guile atomspace needs atomspace of server.
	${ATOMSPACE_smob_LIBRARY}
	${ATOMSPACE_LIBRARY}
	${COGUTIL_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
)

IF (HAVE_CYTHON)
	INSTALL (TARGETS py-shell
		LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog/modules")
ENDIF (HAVE_CYTHON)

INSTALL (TARGETS scheme-shell
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog/modules")

INSTALL (FILES
	GenericShell.h
	DESTINATION "include/${PROJECT_NAME}/cogserver/shell"
)
