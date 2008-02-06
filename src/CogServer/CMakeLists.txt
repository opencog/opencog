SET(CMAKE_BUILD_TYPE Debug)

add_library (CogServer STATIC
CogServerSetup.cc 
CogServer.cc 
SimpleNetworkServer.cc 
ServerSocket.cc 
CommandRequest.cc 
CommandRequestProcessor.cc 
)

include_directories (
${OPENCOG_SOURCE_DIR}/src/Util
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/NMXml
${OPENCOG_SOURCE_DIR}/include/xmlparser
${OPENCOG_SOURCE_DIR}/include/sockets
${OPENCOG_SOURCE_DIR}/include/sockets/openssl
)

link_directories (
${OPENCOG_SOURCE_DIR}/lib/linux/sockets
${OPENCOG_SOURCE_DIR}/lib/linux/xmlparser
)


add_executable (cogServer CogServerMain.cc)

target_link_libraries(CogServer AtomSpace NMXml xmlparser Util Sockets pthread ssl crypto dl)
target_link_libraries (cogServer CogServer)

