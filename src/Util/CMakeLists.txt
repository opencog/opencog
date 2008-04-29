add_library(Util
	exceptions
	lazy_random_selector
    Logger
    mt19937ar
	RandGen
	StringTokenizer
	tree
)

target_link_libraries(Util rt)
