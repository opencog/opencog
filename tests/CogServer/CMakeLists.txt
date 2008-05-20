include_directories (
${OPENCOG_SOURCE_DIR}/src/Util
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/NMXml
${OPENCOG_SOURCE_DIR}/src/CogServer
${OPENCOG_SOURCE_DIR}/include/expat
${OPENCOG_SOURCE_DIR}/include/sockets
${OPENCOG_SOURCE_DIR}/include/sockets/openssl
)

link_directories(
${OPENCOG_BINARY_DIR}/src/AtomSpace
${OPENCOG_BINARY_DIR}/src/NMXml
${OPENCOG_BINARY_DIR}/src/CogServer
${OPENCOG_SOURCE_DIR}/lib/linux/sockets
${OPENCOG_SOURCE_DIR}/lib/linux/expat
)

add_cxxtest(CogServerUTest)
target_link_libraries(CogServerUTest CogServer)
