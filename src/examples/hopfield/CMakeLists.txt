SET(CMAKE_BUILD_TYPE Debug)

include_directories (
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/CogServer
${OPENCOG_SOURCE_DIR}/src/Util
${OPENCOG_SOURCE_DIR}/src/dynamics/attention
)

link_directories (
${OPENCOG_BINARY_DIR}/src/AtomSpace
${OPENCOG_BINARY_DIR}/src/NMXml
${OPENCOG_BINARY_DIR}/src/CogServer
${OPENCOG_BINARY_DIR}/src/dynamics/attention
${OPENCOG_SOURCE_DIR}/lib/linux/sockets
)

add_executable (hopfield HopfieldDemo.cc HopfieldServer.cc Pattern.cc HopfieldOptions.cc)
target_link_libraries (hopfield AtomSpace CogServer Sockets attention)
