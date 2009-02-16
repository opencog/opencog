ADD_LIBRARY (ubigraph SHARED
	UbigraphModule
	Ubigrapher
)

INCLUDE_DIRECTORIES (
	${CSOCKETS_INCLUDE_DIRS}
	${OPENSSL_INCLUDE_DIR}
	${ODBC_INCLUDE_DIRS}
	${UBIGRAPHCLIENT_INCLUDE_DIRS}
)

TARGET_LINK_LIBRARIES(ubigraph
    m
    curl # TODO currently requires making a library called libcurl.so (linked to a specific version) 
    ${UBIGRAPHCLIENT_LIBRARIES} #    ubigraphclient
#    {$XMLRPC_LIBRARIES} #
    xmlrpc_client
    xmlrpc
    xmlrpc_util
    xmlrpc_xmlparse
    xmlrpc_xmltok

)

INSTALL (TARGETS ubigraph
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	UbigraphModule.h
	Ubigrapher.h
DESTINATION "include/opencog/atomspace"
)
