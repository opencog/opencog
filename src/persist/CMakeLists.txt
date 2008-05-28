# Uncomment this if SQL is installed.
# ADD_DEFINITIONS(-DHAVE_SQL_STORAGE)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/AtomSpace
	${OPENCOG_SOURCE_DIR}/src/Util
	${IODBC_INCLUDE_DIRS}
)

ADD_LIBRARY(persist STATIC
	odbcxx.cc
	AtomStorage.cc
)

ADD_EXECUTABLE(sniff
	sniff.cc
)

TARGET_LINK_LIBRARIES(persist iodbc)

TARGET_LINK_LIBRARIES(sniff
	persist
	iodbc
	AtomSpace
	${IODBC_LIBRARIES}
)
