ADD_LIBRARY(Util
	Config
	exceptions
	lazy_random_selector
	Logger
	mt19937ar
	platform.cc
	StringTokenizer
	tree
)
IF (WIN32)
	ADD_LIBRARY(Util getopt.c getopt_long.c)
ENDIF (WIN32)
