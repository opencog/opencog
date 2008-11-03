INCLUDE_DIRECTORIES (
	${CSOCKETS_INCLUDE_DIRS}
	${ODBC_INCLUDE_DIRS}
)

ADD_LIBRARY (builtinreqs SHARED
	BuiltinRequestsModule
	Request
	DataRequest
	ExitRequest
	HelpRequest
	LoadRequest
	LoadModuleRequest
	ListRequest
	ShutdownRequest
	UnloadModuleRequest
)

TARGET_LINK_LIBRARIES(builtinreqs
	xml
	${EXPAT_LIBRARIES}
)

ADD_LIBRARY (server SHARED
	BaseServer
	CogServer
	Request
	NetworkServer
	ConsoleSocket
	GenericShell
#  XXX  need to re-enable loading of files, for Joel.
#	load-file
)

TARGET_LINK_LIBRARIES(server
	util
	atomspace
)

ADD_LIBRARY (scheme-shell SHARED
	SchemeShell
)

TARGET_LINK_LIBRARIES(scheme-shell
	util
	atomspace
	smob
)


# to ease testing/debugging modules on the source tree
IF (CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET_TARGET_PROPERTIES(server PROPERTIES LINK_FLAGS -Wl,-rpath,${OPENCOG_BINARY_DIR}/opencog/dotty:${OPENCOG_BINARY_DIR}/opencog/dynamics/attention:${OPENCOG_BINARY_DIR}/opencog/guile:${OPENCOG_BINARY_DIR}/opencog/nlp/wsd:${OPENCOG_BINARY_DIR}/opencog/persist:${OPENCOG_BINARY_DIR}/opencog/query:${OPENCOG_BINARY_DIR}/opencog/server:${OPENCOG_BINARY_DIR}/opencog/util)
ENDIF (CMAKE_BUILD_TYPE STREQUAL "Debug")

ADD_EXECUTABLE(cogserver
	CogServerMain
)

TARGET_LINK_LIBRARIES(cogserver
	server
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SIGNALS_LIBRARY}
	${CSOCKETS_LIBRARIES}
	${OPENSSL_LIBRARIES}
)

SET_TARGET_PROPERTIES(cogserver PROPERTIES LINK_DIRECTORIES "${BOOST_LIBRARY_DIRS}")
IF (WIN32)
	SET_TARGET_PROPERTIES(cogserver PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libctm")
ENDIF (WIN32)

INSTALL (TARGETS server cogserver builtinreqs
	RUNTIME DESTINATION "bin"
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	Agent.h
	BaseServer.h
	BuiltinRequestsModule.h
	CogServer.h
	ConsoleSocket.h
	DataRequest.h
	ExitRequest.h
	Factory.h
	HelpRequest.h
	IHasMimeType.h
	IRPCSocket.h
	ListRequest.h
	LoadModuleRequest.h
	LoadRequest.h
	Module.h
	NetworkServer.h
	Registry.h
	Request.h
	ShutdownRequest.h
	UnloadModuleRequest.h
	DESTINATION "include/opencog/server"
)
