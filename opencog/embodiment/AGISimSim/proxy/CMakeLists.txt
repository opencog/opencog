INCLUDE_DIRECTORIES(
    ${OPENCOG_SOURCE_DIR}/opencog
)

ADD_LIBRARY(AGISimSimProxy
    SimProxy.cc
    SimClientSocket.cc
    DOMPrintErrorHandler.cc
    DOMTreeErrorReporter.cc 
)

TARGET_LINK_LIBRARIES(AGISimSimProxy
    util
    ssl
    crypto
    dl
    xerces-c
)
