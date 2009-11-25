# To make Boost ublas much more efficient.
ADD_DEFINITIONS(-DNDEBUG)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/atomspace
	${OPENCOG_BINARY_DIR}/opencog/xml
	${OPENCOG_BINARY_DIR}/opencog/server
	${OPENCOG_BINARY_DIR}/opencog/dynamics/attention
)

INCLUDE_DIRECTORIES (
	${ODBC_INCLUDE_DIRS}
)

IF (XMLRPC_FOUND AND CURL_FOUND)
    SET (HOPFIELD_UBI_SRC HopfieldUbigrapher.cc)
    SET (OPENCOG_UBI_LIB ubigraph
        ${XMLRPC_LIBRARIES}
        )
ENDIF (XMLRPC_FOUND AND CURL_FOUND)

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
	${EXPAT_LIBRARIES}
    ${OPENCOG_UBI_LIB}
)
