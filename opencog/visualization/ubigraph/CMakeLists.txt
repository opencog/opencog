SET(CMAKE_C_FLAGS_DEBUG "-O0 -g")

# Arghhh this results in a circular dependency between pln and ubigraph.
# The solution is to move BITUbigrapher out of this directory, and over
# to the PLN directory.
#IF (HAVE_PLN)
#	SET(BITUBI BITUbigrapher)
#	SET(PLN_LIB pln)
#ENDIF (HAVE_PLN)


ADD_LIBRARY (ubigraph SHARED
	UbigraphModule
	Ubigrapher
	client
	${BITUBI}
)

ADD_DEPENDENCIES(ubigraph opencog_atom_types)

INCLUDE_DIRECTORIES (
	${ODBC_INCLUDE_DIRS}
)

TARGET_LINK_LIBRARIES(ubigraph
	${PLN_LIB}
	m
	${CURL_LIBRARIES} 
	${XMLRPC_LIBRARIES}
)

INSTALL (TARGETS ubigraph
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	UbigraphAPI.h
	UbigraphModule.h
	Ubigrapher.h
	DESTINATION "include/${PROJECT_NAME}/ubigraph"
)
