
add_library (query STATIC
QueryProcessor.cc
)

include_directories (
${OPENCOG_SOURCE_DIR}/src/CogServer
)
