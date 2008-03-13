add_library (persist STATIC
odbcxx.cc
)

include_directories (
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/Util
)

add_executable (sniff odbcxx.cc)

target_link_libraries (sniff iodbc)
