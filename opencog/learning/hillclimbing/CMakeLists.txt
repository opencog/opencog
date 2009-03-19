INCLUDE_DIRECTORIES(
    ${OPENCOG_SOURCE_DIR}/opencog
)

LINK_DIRECTORIES(
    ${OPENCOG_BINARY_DIR}/opencog/util
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
)

ADD_LIBRARY(hillclimbing
	hillclimber
	ant-hillclimber
	NeighborhoodGenerator
)

TARGET_LINK_LIBRARIES(hillclimbing
	comboreduct
	util
)

ADD_SUBDIRECTORY (main)
