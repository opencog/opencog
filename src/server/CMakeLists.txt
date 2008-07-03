SET(CMAKE_BUILD_TYPE Debug)

ADD_LIBRARY (server STATIC
	CogServer.cc 
	SimpleNetworkServer.cc 
	ServerSocket.cc 
	CommandRequest.cc 
	CommandRequestProcessor.cc 
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/util
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/xml
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/guile
	${OPENCOG_SOURCE_DIR}/src/memcache
	${OPENCOG_SOURCE_DIR}/src/nlp/wsd
	${OPENCOG_SOURCE_DIR}/src/persist
	${OPENCOG_SOURCE_DIR}/src/query
	${CSOCKETS_INCLUDE_DIRS}
	${OPENSSL_INCLUDE_DIR}
	${ODBC_INCLUDE_DIRS}
	${LIBMEMCACHED_INCLUDE_DIRS}
)

ADD_EXECUTABLE(cogserver CogServerMain.cc)

TARGET_LINK_LIBRARIES(server
	atomspace
	xml
	util
	query
	schemes
	wsd
	persist
	${ODBC_LIBRARIES}
	${CSOCKETS_LIBRARIES}
	${OPENSSL_LIBRARIES}
	${EXPAT_LIBRARIES}
	${GUILE_LIBRARIES}
	${LIBMEMCACHED_LIBRARIES}
)

TARGET_LINK_LIBRARIES(cogserver server)
IF (WIN32)
	SET_TARGET_PROPERTIES(cogserver PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libctm")
ENDIF (WIN32)
