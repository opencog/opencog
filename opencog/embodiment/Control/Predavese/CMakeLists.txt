IF(NOT WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
ENDIF(NOT WIN32)

ADD_LIBRARY(Predavese 
    Predavese.cc
    PredaveseParser.cc
    PredaveseActions.cc
    NominalReferenceResolver.cc
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}
    ${OPENCOG_SOURCE_DIR}/lib/linux/xerces
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
)

INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
    ${OPENCOG_SOURCE_DIR}/test/Control/PerceptionActionInterface
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Learning
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetComboVocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
)

TARGET_LINK_LIBRARIES (Predavese
    util
    xerces-c
    comboreduct
    WorldWrapper
	${OPENCOG_LIBRARIES}
)
