
IF (HAVE_SERVER)
	ADD_SUBDIRECTORY(benchmark)
	ADD_SUBDIRECTORY(python)
	IF (HAVE_ZMQ AND TBB_FOUND)
		ADD_SUBDIRECTORY(events)
	ENDIF (HAVE_ZMQ AND TBB_FOUND)
ENDIF (HAVE_SERVER)

# ----------------------------------------
IF (TBB_FOUND)
	INCLUDE_DIRECTORIES (
		${TBB_INCLUDE_DIR}
	)

	LINK_DIRECTORIES(
		${TBB_INCLUDE_DIR}
		${TBB_LIBRARY_DIRS}
	)

	ADD_LIBRARY(plnmodule SHARED
		PLNModule
	)

	TARGET_LINK_LIBRARIES(plnmodule
		ruleengine
		server
		tbb
	)
ENDIF (TBB_FOUND)

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

# ----------------------------------------
# Build the RuleEngine cogserver module

ADD_LIBRARY(RuleEngineModule SHARED
	RuleEngineModule.cc
)

TARGET_LINK_LIBRARIES(RuleEngineModule ruleengine server)

# ----------------------------------------
ADD_LIBRARY (SuRealModule SHARED
	SuRealModule
)

ADD_DEPENDENCIES (SuRealModule
	nlp_atom_types
)

TARGET_LINK_LIBRARIES(SuRealModule
	sureal
)
