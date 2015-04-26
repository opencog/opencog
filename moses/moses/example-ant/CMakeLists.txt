# We need to put this in a library, so that AntUTest can link to it.
ADD_LIBRARY (ant_scoring
	ant_scoring
)

ADD_EXECUTABLE (moses-ant-hillclimbing
	moses-ant-hillclimbing
	ant_scoring
)

TARGET_LINK_LIBRARIES (moses-ant-hillclimbing
	moses
	comboreduct
	comboant
	cogutil
)
