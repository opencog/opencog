INCLUDE_DIRECTORIES(
    ${PYTHON_INCLUDE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

ADD_LIBRARY (py-shell SHARED
        GenericShell
        PythonShell
        PythonShellModule
)

TARGET_LINK_LIBRARIES(py-shell
        smob
        server
        atomspace
        cogutil
        ${Boost_SYSTEM_LIBRARY}
        ${PYTHON_LIBRARIES}
)

ADD_LIBRARY (scheme-shell SHARED
	GenericShell
	SchemeShell
	SchemeShellModule
)

TARGET_LINK_LIBRARIES(scheme-shell
	smob
	server
	atomspace
	cogutil
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

