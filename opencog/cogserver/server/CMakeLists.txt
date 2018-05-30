INCLUDE_DIRECTORIES (${GUILE_INCLUDE_DIRS})

IF (HAVE_CYTHON)
	INCLUDE_DIRECTORIES (${PYTHON_INCLUDE_DIRS})
ENDIF (HAVE_CYTHON)


ADD_LIBRARY (builtinreqs SHARED
	BuiltinRequestsModule
	Request
	LoadModuleRequest
	ListRequest
	ShutdownRequest
	UnloadModuleRequest
	ListModulesRequest
)

TARGET_LINK_LIBRARIES(builtinreqs
	server
)

INSTALL (TARGETS builtinreqs
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog/modules")

# ------------------------------------------------------------

ADD_LIBRARY (server SHARED
	Agent
	AgentRunnerBase
	AgentRunnerThread
	BaseServer
	CogServer
	Request
	NetworkServer
	ServerSocket
	ConsoleSocket
	SystemActivityTable
)

TARGET_LINK_LIBRARIES(server
	${ATOMSPACE_attentionbank_LIBRARY}
	${ATOMSPACE_execution_LIBRARY}
	${ATOMSPACE_smob_LIBRARY}
	${COGUTIL_LIBRARY}
	${Boost_FILESYSTEM_LIBRARY}
	dl
)

IF (HAVE_CYTHON)
	TARGET_LINK_LIBRARIES(server ${ATOMSPACE_PythonEval_LIBRARY})
ENDIF (HAVE_CYTHON)

# IF (HAVE_GUILE)
# 	TARGET_LINK_LIBRARIES(server scheme-shell ${ATOMSPACE_LIBRARIES})
# ENDIF (HAVE_GUILE)

# ------------------------------------------------------------
# This defines the guile scheme loadable module

IF (HAVE_GUILE)
	ADD_LIBRARY (guile-cogserver SHARED
		CogServerSCM
	)

	TARGET_LINK_LIBRARIES(guile-cogserver
		server
		${Boost_SYSTEM_LIBRARY}
		${GUILE_LIBRARIES}
	)

	INSTALL (TARGETS guile-cogserver DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (HAVE_GUILE)

# ------------------------------------------------------------

ADD_EXECUTABLE(cogserver
	CogServerMain
)

TARGET_LINK_LIBRARIES(cogserver
	server
	${Boost_SYSTEM_LIBRARY}
)

SET_TARGET_PROPERTIES(cogserver PROPERTIES LINK_DIRECTORIES "${BOOST_LIBRARY_DIRS}")
IF (WIN32)
	SET_TARGET_PROPERTIES(cogserver PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libctm")
ENDIF (WIN32)

INSTALL (TARGETS cogserver RUNTIME DESTINATION "bin")

INSTALL (TARGETS server DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

INSTALL (FILES
	Agent.h
	BaseServer.h
	CogServer.h
	ConsoleSocket.h
	Factory.h
	Module.h
	NetworkServer.h
	SystemActivityTable.h
	Registry.h
	Request.h
	RequestClassInfo.h
	DESTINATION "include/${PROJECT_NAME}/cogserver/server"
)
