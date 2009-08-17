IF(NOT WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
ENDIF(NOT WIN32)

ADD_LIBRARY(Predavese 
    PorterStemmer.cc
    Predavese.cc
    PredaveseParser.cc
    PredaveseActions.cc
    NominalReferenceResolver.cc
)

TARGET_LINK_LIBRARIES (Predavese
    ${XERCESC_LIBRARIES}
    WorldWrapper
)
