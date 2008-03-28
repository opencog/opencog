SET(CMAKE_BUILD_TYPE Debug)

include_directories (
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/CogServer
${OPENCOG_SOURCE_DIR}/src/Util
)

add_executable (hopfield HopfieldServer.cc)

target_link_libraries(hopfield CogServer Sockets)


