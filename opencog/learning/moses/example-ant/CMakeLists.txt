ADD_EXECUTABLE (moses-ant-hillclimbing
    moses-ant-hillclimbing
    ant_scoring
)

TARGET_LINK_LIBRARIES (moses-ant-hillclimbing
	moses
	comboreduct
	cogutil
)
