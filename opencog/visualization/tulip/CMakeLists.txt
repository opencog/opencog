ADD_LIBRARY(adaptor-tulip STATIC
    TulipWriter.cc
)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/opencog/util
    ${PROJECT_SOURCE_DIR}/opencog/atomspace
    ${PROJECT_SOURCE_DIR}/opencog/server
)

TARGET_LINK_LIBRARIES(adaptor-tulip server)
