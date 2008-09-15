# Uncomment this if SQL is installed.
# ADD_DEFINITIONS(-DHAVE_SQL_STORAGE)

INCLUDE_DIRECTORIES (
	${ODBC_INCLUDE_DIRS}
)

ADD_LIBRARY(persist STATIC
	odbcxx.cc
	AtomStorage.cc
)

ADD_EXECUTABLE(sniff
	sniff.cc
)

TARGET_LINK_LIBRARIES(persist
	${ODBC_LIBRARIES}
)

TARGET_LINK_LIBRARIES(sniff
	persist
	atomspace
	${ODBC_LIBRARIES}
)
