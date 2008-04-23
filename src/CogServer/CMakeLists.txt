SET(CMAKE_BUILD_TYPE Debug)

add_library (CogServer STATIC
CogServerSetup.cc 
CogServer.cc 
SimpleNetworkServer.cc 
ServerSocket.cc 
CommandRequest.cc 
CommandRequestProcessor.cc 
ImportanceUpdatingAgent.cc
ImportanceSpreadingAgent.cc
HebbianLearningAgent.cc
)

include_directories (
${OPENCOG_SOURCE_DIR}/src/Util
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/NMXml
${OPENCOG_SOURCE_DIR}/src/CogServer
${OPENCOG_SOURCE_DIR}/src/query
#
# Local copy of xml parser, should be eliminated.
# ${OPENCOG_SOURCE_DIR}/include/xmlparser
#
# Local copy of http://www.alhem.net/Sockets/
${OPENCOG_SOURCE_DIR}/include/sockets
#
# Local copy of openssl, should be eliminated.
# ${OPENCOG_SOURCE_DIR}/include/sockets/openssl
)

link_directories (
${OPENCOG_SOURCE_DIR}/lib/linux/sockets
# local copy of the xml parser, should be eliminated.
# ${OPENCOG_SOURCE_DIR}/lib/linux/xmlparser
)


add_executable (cogServer CogServerMain.cc)

target_link_libraries(CogServer AtomSpace query NMXml xmlparse Util Sockets pthread ssl crypto dl)
target_link_libraries (cogServer CogServer)

