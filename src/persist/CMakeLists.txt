add_library (persist STATIC
odbcxx.cc
AtomStorage.cc
)

include_directories (
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/Util
)



add_executable (sniff sniff.cc)

target_link_libraries (persist iodbc)

target_link_libraries (sniff persist iodbc AtomSpace)
