# To make Boost ublas much more efficient.
ADD_DEFINITIONS(-DNDEBUG)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/src/atomspace
	${OPENCOG_BINARY_DIR}/src/xml
	${OPENCOG_BINARY_DIR}/src/server
	${OPENCOG_BINARY_DIR}/src/dynamics/attention
)

ADD_EXECUTABLE (hopfield
	HopfieldDemo.cc
	HopfieldServer.cc
	Pattern.cc
	HopfieldOptions.cc
	StorkeyAgent.cc
    ImprintAgent.cc
)

TARGET_LINK_LIBRARIES (hopfield
	server
	attention
	${Boost_SIGNALS_LIBRARY}
#	${GSL_LIBRARIES}
	${CSOCKETS_LIBRARIES}
	${OPENSSL_LIBRARIES}
	${EXPAT_LIBRARIES}
)
