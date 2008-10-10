ADD_LIBRARY(adaptor-tulip STATIC
    TulipWriter.cc
)

INCLUDE_DIRECTORIES(
    ${OPENCOG_SOURCE_DIR}/src/util
    ${OPENCOG_SOURCE_DIR}/src/atomspace
    ${OPENCOG_SOURCE_DIR}/src/server
)

TARGET_LINK_LIBRARIES(adaptor-tulip server)
