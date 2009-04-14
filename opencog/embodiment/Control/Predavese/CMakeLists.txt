IF(NOT WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
ENDIF(NOT WIN32)

ADD_LIBRARY(Predavese 
    Predavese.cc
    PredaveseParser.cc
    PredaveseActions.cc
    NominalReferenceResolver.cc
)

INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog)

TARGET_LINK_LIBRARIES (Predavese
    util
    ${XERCESC_LIBRARIES}
    comboreduct
    WorldWrapper
)
