INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/learning/hillclimbing
    ${OPENCOG_SOURCE_DIR}/opencog/learning/moses
    ${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/util
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/learning/moses
    ${OPENCOG_BINARY_DIR}/opencog/learning/hillclimbing
)

ADD_EXECUTABLE (ant-hillclimbing ant-hillclimbing)
TARGET_LINK_LIBRARIES (ant-hillclimbing
	hillclimbing
)
