INCLUDE_DIRECTORIES (
	${GUILE_INCLUDE_DIRS}
)

ADD_LIBRARY (builtinreqs SHARED
	BuiltinRequestsModule
	Request
	DataRequest
	ExitRequest
	SleepRequest
	HelpRequest
	LoadRequest
	SaveRequest
	LoadModuleRequest
	ListRequest
	ShutdownRequest
	UnloadModuleRequest
)

TARGET_LINK_LIBRARIES(builtinreqs util)

IF (EXPAT_FOUND)
	TARGET_LINK_LIBRARIES(builtinreqs xml)
ENDIF (EXPAT_FOUND)


# XXX FIXME TODO Until such time that the learning-server messages are
# removed from the spacemap, we have to build static, to avoid circular
# dependencies. That's just plain wrong.
#ADD_LIBRARY (server SHARED
ADD_LIBRARY (server STATIC
	BaseServer
	CogServer
	Request
	NetworkServer
	ServerSocket
	ConsoleSocket
	Agent
	SystemActivityTable
)

SET(SERVER_LINK_LIBS 
	builtinreqs
	atomspace
	space_server
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_THREAD_LIBRARY}
	dl
)

IF (EXPAT_FOUND)
	SET(SERVER_LINK_LIBS xml ${SERVER_LINK_LIBS})
ENDIF (EXPAT_FOUND)

IF (HAVE_GUILE)
	SET(SERVER_LINK_LIBS smob ${SERVER_LINK_LIBS})
ENDIF (HAVE_GUILE)

IF (ODBC_FOUND)
	SET(SERVER_LINK_LIBS persist ${SERVER_LINK_LIBS})
ENDIF (ODBC_FOUND)

TARGET_LINK_LIBRARIES(server ${SERVER_LINK_LIBS})

ADD_EXECUTABLE(cogserver
	CogServerMain
)

TARGET_LINK_LIBRARIES(cogserver
	server
	${Boost_SYSTEM_LIBRARY}
	${GUILE_LIBRARIES}
)

# to ease testing/debugging modules on the source tree
# IF (CMAKE_BUILD_TYPE STREQUAL "Debug")
# XXX what is this shit?  The below is one wild and insane violation of everything that
# cmake stands for ... why bother using cmake, if we are going to crap on every principle
# of intelligent makefile design.  WTF. 
IF (APPLE)
	# We should eventually do Apple builds with "packages"
	# Apple's linker seems not to understand a colon-separated list of
	# rpaths.
	SET_TARGET_PROPERTIES(server PROPERTIES LINK_FLAGS
		"-Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/dotty -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/ubigraph -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/dynamics/attention -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/guile  -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/learning/dimensionalembedding -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/nlp/wsd -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/persist/sql -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/query -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/server -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/shell -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/util -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/web -Wl,-rpath,/opt/local/lib -Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/cython/")
	# doesn't work for some reason...
	#SET_TARGET_PROPERTIES(cogserver PROPERTIES INSTALL_RPATH
	#	${PROJECT_BINARY_DIR}/opencog/guile)
	#SET_TARGET_PROPERTIES(cogserver PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
ELSE(APPLE)
	SET_TARGET_PROPERTIES(server PROPERTIES LINK_FLAGS
		-Wl,-rpath,${PROJECT_BINARY_DIR}/opencog/dotty:${PROJECT_BINARY_DIR}/opencog/ubigraph:${PROJECT_BINARY_DIR}/opencog/dynamics/attention:${PROJECT_BINARY_DIR}/opencog/guile:${PROJECT_BINARY_DIR}/opencog/learning/dimensionalembedding:${PROJECT_BINARY_DIR}/opencog/nlp/wsd:${PROJECT_BINARY_DIR}/opencog/persist:${PROJECT_BINARY_DIR}/opencog/query:${PROJECT_BINARY_DIR}/opencog/server:${PROJECT_BINARY_DIR}/opencog/shell:${PROJECT_BINARY_DIR}/opencog/util:${PROJECT_BINARY_DIR}/opencog/web:${PROJECT_BINARY_DIR}/opencog/cython/)
ENDIF (APPLE)
#ENDIF (CMAKE_BUILD_TYPE STREQUAL "Debug")

SET_TARGET_PROPERTIES(cogserver PROPERTIES LINK_DIRECTORIES "${BOOST_LIBRARY_DIRS}")
IF (WIN32)
	SET_TARGET_PROPERTIES(cogserver PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libctm")
ENDIF (WIN32)

INSTALL (TARGETS cogserver builtinreqs
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
	IRequestComplete.h
	ListRequest.h
	LoadModuleRequest.h
	LoadRequest.h
	SaveRequest.h
	Module.h
	NetworkServer.h
	SocketListener.h
	SocketPort.h
	SystemActivityTable.h
	Registry.h
	Request.h
	RequestResult.h
	ShutdownRequest.h
	UnloadModuleRequest.h
	DESTINATION "include/${PROJECT_NAME}/server"
)
