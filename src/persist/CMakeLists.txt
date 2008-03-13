add_library (persist STATIC
odbc.cc
)

include_directories (
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/Util
)

add_executable (sniff odbc.cc)

target_link_libraries (sniff iodbc)
