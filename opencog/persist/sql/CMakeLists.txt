# Uncomment this if SQL is installed.
# ADD_DEFINITIONS(-DHAVE_SQL_STORAGE)

INCLUDE_DIRECTORIES (
	${ODBC_INCLUDE_DIRS}
)

ADD_LIBRARY (persist-sql SHARED
	odbcxx.cc
	AtomStorage.cc
	SQLPersistSCM.cc
)

ADD_LIBRARY (PersistModule SHARED
	PersistModule.cc
)

# XXX FIXME We've added a depedency on the NLP atom types.
# this should be removed when the NLP_HACK in PersistModule is removed.
ADD_DEPENDENCIES(persist-sql opencog_atom_types)
ADD_DEPENDENCIES(persist-sql nlp_atom_types)

TARGET_LINK_LIBRARIES(persist-sql
	atomspace
	nlp-types
	${ODBC_LIBRARIES}
)

IF (HAVE_GUILE)
	TARGET_LINK_LIBRARIES(persist-sql smob)
ENDIF (HAVE_GUILE)

TARGET_LINK_LIBRARIES(PersistModule
	persist
	persist-sql
)

ADD_EXECUTABLE(sniff
	sniff.cc
)

TARGET_LINK_LIBRARIES(sniff
	persist-sql
	atomspace
	${ODBC_LIBRARIES}
)

INSTALL (TARGETS persist-sql
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)
INSTALL (TARGETS PersistModule
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)
