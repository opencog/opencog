include_directories(
${OPENCOG_SOURCE_DIR}/src/Util
)

link_directories(
${OPENCOG_BINARY_DIR}
${OPENCOG_BINARY_DIR}/src/Util
)

add_cxxtest(LoggerUTest)
target_link_libraries(LoggerUTest Util)
IF(WIN32)
target_link_libraries(LoggerUTest winmm)
ENDIF(WIN32)

add_cxxtest(StringTokenizerUTest)
target_link_libraries(StringTokenizerUTest Util)
IF(WIN32)
target_link_libraries(StringTokenizerUTest winmm)
ENDIF(WIN32)
