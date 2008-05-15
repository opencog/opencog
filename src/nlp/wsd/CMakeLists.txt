
add_library (wsd STATIC
Mihalcea.cc
MihalceaLabel.cc
WordSenseProcessor.cc
)

include_directories (
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/CogServer
${OPENCOG_SOURCE_DIR}/src/Util
)
