# Uncomment this if SQL is installed.
# ADD_DEFINITIONS(-DHAVE_SQL_STORAGE)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/util
	${IODBC_INCLUDE_DIRS}
)

ADD_LIBRARY(persist STATIC
	odbcxx.cc
	AtomStorage.cc
)

ADD_EXECUTABLE(sniff
	sniff.cc
)

TARGET_LINK_LIBRARIES(persist
	${IODBC_LIBRARIES}
)

TARGET_LINK_LIBRARIES(sniff
	persist
	atomspace
	${IODBC_LIBRARIES}
)
