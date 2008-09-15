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
)

TARGET_LINK_LIBRARIES (hopfield
	server
	attention
	${CSOCKETS_LIBRARIES}
	gsl
    gslcblas
)
