INCLUDE(FindXMLRPC)

FIND_PACKAGE(XMLRPC) # REQUIRED curl-client)

IF (XMLRPC_FOUND)

ADD_LIBRARY (ubigraph SHARED
	UbigraphModule 
)

INCLUDE_DIRECTORIES (
	${CSOCKETS_INCLUDE_DIRS}
	${OPENSSL_INCLUDE_DIR}
	${ODBC_INCLUDE_DIRS}
)

TARGET_LINK_LIBRARIES(ubigraph
    m
    curl # TODO currently requires making a library called libcurl.so (linked to a specific version) 
    ubigraphclient
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

ENDIF (XMLRPC_FOUND)

