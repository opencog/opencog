INCLUDE_DIRECTORIES(
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/WorldWrapper
)
LINK_DIRECTORIES(
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/WorldWrapper
)


ADD_CXXTEST(WorldWrapperUtilCacheUTest)
TARGET_LINK_LIBRARIES(WorldWrapperUtilCacheUTest
    WorldWrapper
    ${XERCESC_LIBRARIES}
)

