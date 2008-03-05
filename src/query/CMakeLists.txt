
add_library (query STATIC
FrameQuery.cc
QueryProcessor.cc
PatternMatch.cc
RelexQuery.cc
)

include_directories (
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/CogServer
${OPENCOG_SOURCE_DIR}/src/Util
)
