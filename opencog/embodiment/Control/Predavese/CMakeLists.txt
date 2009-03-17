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
    ${PETAVERSE_BINARY_DIR}
    ${PETAVERSE_SOURCE_DIR}/lib/linux/xerces
    ${PETAVERSE_BINARY_DIR}/src/WorldWrapper
    ${PETAVERSE_SOURCE_DIR}/src/PetComboVocabulary
)

INCLUDE_DIRECTORIES (
    ${PETAVERSE_SOURCE_DIR}/src/Control/Predavese
    ${PETAVERSE_SOURCE_DIR}/test/Control/PerceptionActionInterface
    ${PETAVERSE_SOURCE_DIR}/src/Control
	${PETAVERSE_SOURCE_DIR}/src/AtomSpaceExtensions
    ${PETAVERSE_SOURCE_DIR}/src/Spatial
    ${PETAVERSE_SOURCE_DIR}/src/Spatial/Math
    ${PETAVERSE_SOURCE_DIR}/src/Learning
    ${PETAVERSE_SOURCE_DIR}/src/PetComboVocabulary
    ${PETAVERSE_SOURCE_DIR}/src/WorldWrapper
    ${PETAVERSE_SOURCE_DIR}/src/Control/MessagingSystem
)

TARGET_LINK_LIBRARIES (Predavese
    ${LADSUTIL_LIBRARY}
    xerces-c
    ${COMBOREDUCT_LIBRARY}
    WorldWrapper
	${OPENCOG_LIBRARIES}
)
