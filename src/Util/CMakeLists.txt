ADD_LIBRARY(Util
	Config
	exceptions
	lazy_random_selector
    Logger
    mt19937ar
	RandGen
	StringTokenizer
	tree
)

TARGET_LINK_LIBRARIES(Util
	rt
)
