INCLUDE_DIRECTORIES(
    ${OPENCOG_SOURCE_DIR}/opencog
    ${OPENCOG_SOURCE_DIR}/opencog/comboreduct
    ${OPENCOG_SOURCE_DIR}/opencog/comboreduct/ant_combo_vocabulary
    ${OPENCOG_SOURCE_DIR}/opencog/learning/hillclimbing/
)
LINK_DIRECTORIES(
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/learning/hillclimbing
    ${OPENCOG_BINARY_DIR}/opencog/util
)

ADD_CXXTEST(NeighborhoodGeneratorUTest)
TARGET_LINK_LIBRARIES(NeighborhoodGeneratorUTest
    hillclimbing
    comboreduct
    util
)
