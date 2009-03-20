# To make Boost ublas much more efficient.
ADD_DEFINITIONS(-DNDEBUG)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/src/atomspace
	${OPENCOG_BINARY_DIR}/src/xml
	${OPENCOG_BINARY_DIR}/src/server
	${OPENCOG_BINARY_DIR}/src/dynamics/attention
)

INCLUDE_DIRECTORIES (
	${CSOCKETS_INCLUDE_DIRS}
	${OPENSSL_INCLUDE_DIR}
	${ODBC_INCLUDE_DIRS}
	${UBIGRAPHCLIENT_INCLUDE_DIRS}
)

ADD_EXECUTABLE (hopfield
	HopfieldDemo.cc
	HopfieldServer.cc
	Pattern.cc
	HopfieldOptions.cc
    HopfieldUbigrapher.cc
	StorkeyAgent.cc
    ImprintAgent.cc
)

ADD_DEPENDENCIES(hopfield attention_atom_types)

TARGET_LINK_LIBRARIES (hopfield
	server
	attention
    ubigraph
	${Boost_SIGNALS_LIBRARY}
#	${GSL_LIBRARIES}
	${CSOCKETS_LIBRARIES}
	${OPENSSL_LIBRARIES}
	${EXPAT_LIBRARIES}
    curl # TODO currently requires making a library called libcurl.so (linked to a specific version) 
    ${UBIGRAPHCLIENT_LIBRARIES} #    ubigraphclient
#    {$XMLRPC_LIBRARIES} #
    xmlrpc_client
    xmlrpc
    xmlrpc_util
    xmlrpc_xmlparse
    xmlrpc_xmltok
)
