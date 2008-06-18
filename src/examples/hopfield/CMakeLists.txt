INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/AtomSpace
	${OPENCOG_SOURCE_DIR}/src/CogServer
	${OPENCOG_SOURCE_DIR}/src/Util
	${OPENCOG_SOURCE_DIR}/src/dynamics/attention
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/src/AtomSpace
	${OPENCOG_BINARY_DIR}/src/NMXml
	${OPENCOG_BINARY_DIR}/src/CogServer
	${OPENCOG_BINARY_DIR}/src/dynamics/attention
)

ADD_EXECUTABLE (hopfield
	HopfieldDemo.cc
	HopfieldServer.cc
	Pattern.cc
	HopfieldOptions.cc
)

TARGET_LINK_LIBRARIES (hopfield
	AtomSpace
	Server
	attention
	${CSOCKETS_LIBRARIES}
)
