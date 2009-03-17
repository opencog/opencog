INCLUDE_DIRECTORIES(
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
)
LINK_DIRECTORIES(
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
    ${OPENCOG_SOURCE_DIR}/lib/linux/xerces
)


ADD_CXXTEST(WorldWrapperUtilCacheUTest)
TARGET_LINK_LIBRARIES(WorldWrapperUtilCacheUTest
    WorldWrapper
)

