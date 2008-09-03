IF (WIN32)
	SET(WIN32_GETOPT_FILES getopt getopt_long)
ELSE (WIN32)
	SET(WIN32_GETOPT_FILES)
ENDIF (WIN32)

ADD_LIBRARY(util
	Config
	exceptions
	Logger
	mt19937ar
	platform.cc
	StringTokenizer
	tree
	${WIN32_GETOPT_FILES}
)
