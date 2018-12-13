# To make Boost ublas much more efficient.
ADD_DEFINITIONS(-DNDEBUG)

LINK_DIRECTORIES (
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/atoms/truthvalue
	${PROJECT_BINARY_DIR}/opencog/cogserver/server
	${PROJECT_BINARY_DIR}/opencog/attention
)

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES (
	${CMAKE_BINARY_DIR}
	${ODBC_INCLUDE_DIRS}
)

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
	${ATOMSPACE_LIBRARIES}
	server
	attention
)
