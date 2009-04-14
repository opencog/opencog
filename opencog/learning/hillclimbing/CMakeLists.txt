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
