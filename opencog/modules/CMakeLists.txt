
IF (HAVE_SERVER)
	ADD_SUBDIRECTORY(benchmark)
	IF (HAVE_ZMQ AND TBB_FOUND)
		ADD_SUBDIRECTORY(events)
	ENDIF (HAVE_ZMQ AND TBB_FOUND)
ENDIF (HAVE_SERVER)

# ----------------------------------------
# Build the query (pattern-matching) cogserver module
ADD_LIBRARY(QueryModule SHARED
	QueryModule.cc
)
TARGET_LINK_LIBRARIES(QueryModule query server)

# ----------------------------------------
# The SQL persistance module

ADD_LIBRARY (PersistModule SHARED
	PersistModule.cc
)

TARGET_LINK_LIBRARIES(PersistModule
	persist
	persist-sql
)

INSTALL (TARGETS PersistModule
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)
