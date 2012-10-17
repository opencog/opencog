# Uncomment this if SQL is installed.
# ADD_DEFINITIONS(-DHAVE_SQL_STORAGE)

INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/src/server
	${ODBC_INCLUDE_DIRS}
)

ADD_LIBRARY (persist SHARED
	odbcxx.cc
	AtomStorage.cc
	PersistModule.cc
)

ADD_DEPENDENCIES(persist opencog_atom_types)

TARGET_LINK_LIBRARIES(persist
	atomspace
	builtinreqs
	${ODBC_LIBRARIES}
)

IF (HAVE_GUILE)
	TARGET_LINK_LIBRARIES(persist smob)
ENDIF (HAVE_GUILE)

ADD_EXECUTABLE(sniff
	sniff.cc
)

TARGET_LINK_LIBRARIES(sniff
	persist
	SpaceMap
	space_server
	server
	atomspace
	${Boost_SIGNALS_LIBRARY}
	${ODBC_LIBRARIES}
)

INSTALL (TARGETS persist
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	AtomStorage.h
	odbcxx.h
	PersistModule.h
	DESTINATION "include/${PROJECT_NAME}/persist/sql"
)
