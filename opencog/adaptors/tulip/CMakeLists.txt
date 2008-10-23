ADD_LIBRARY(adaptor-tulip STATIC
    TulipWriter.cc
)

INCLUDE_DIRECTORIES(
    ${OPENCOG_SOURCE_DIR}/opencog/util
    ${OPENCOG_SOURCE_DIR}/opencog/atomspace
    ${OPENCOG_SOURCE_DIR}/opencog/server
)

TARGET_LINK_LIBRARIES(adaptor-tulip server)
