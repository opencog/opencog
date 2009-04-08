INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/
    ${OPENCOG_SOURCE_DIR}/opencog/learning/
    ${OPENCOG_SOURCE_DIR}/opencog/learning/pleasure
)

LINK_DIRECTORIES(
    ${OPENCOG_BINARY_DIR}/opencog/learning/pleasure
    ${OPENCOG_BINARY_DIR}/opencog/util
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
)

ADD_EXECUTABLE (main main)
TARGET_LINK_LIBRARIES (main
	pleasure
        comboreduct
        util
)

ADD_CUSTOM_COMMAND(
    TARGET main
    POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy ${OPENCOG_SOURCE_DIR}/opencog/learning/pleasure/main/nodelist ${OPENCOG_BINARY_DIR}/opencog/learning/pleasure/main
)


