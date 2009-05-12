# To make Boost ublas much more efficient.
ADD_DEFINITIONS(-DNDEBUG)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/atomspace
	${OPENCOG_BINARY_DIR}/opencog/xml
	${OPENCOG_BINARY_DIR}/opencog/server
	${OPENCOG_BINARY_DIR}/opencog/dynamics/attention
)

INCLUDE_DIRECTORIES (
	${CSOCKETS_INCLUDE_DIRS}
	${OPENSSL_INCLUDE_DIR}
	${ODBC_INCLUDE_DIRS}
	${UBIGRAPHCLIENT_INCLUDE_DIRS}
)

IF (UBIGRAPHCLIENT_FOUND AND XMLRPC_FOUND)
    SET (HOPFIELD_UBI_SRC HopfieldUbigrapher.cc)
    SET (OPENCOG_UBI_LIB ubigraph
        ${UBIGRAPHCLIENT_LIBRARIES}
#       {$XMLRPC_LIBRARIES}
        ${XMLRPC_LIBRARIES}
        #xmlrpc_client
        #xmlrpc
        #xmlrpc_util
        #xmlrpc_xmlparse
        #xmlrpc_xmltok
        )
ENDIF (UBIGRAPHCLIENT_FOUND AND XMLRPC_FOUND)

ADD_EXECUTABLE (hopfield
	HopfieldDemo.cc
	HopfieldServer.cc
	Pattern.cc
	HopfieldOptions.cc
	StorkeyAgent.cc
    ImprintAgent.cc
    ${HOPFIELD_UBI_SRC}
)

ADD_DEPENDENCIES(hopfield attention_atom_types)

TARGET_LINK_LIBRARIES (hopfield
	server
	attention
	${Boost_SIGNALS_LIBRARY}
	${CSOCKETS_LIBRARIES}
	${OPENSSL_LIBRARIES}
	${EXPAT_LIBRARIES}
    ${OPENCOG_UBI_LIB}
)
