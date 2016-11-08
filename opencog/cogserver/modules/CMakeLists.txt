
IF (HAVE_SERVER)
	ADD_SUBDIRECTORY(benchmark)
	IF (HAVE_CYTHON)
		ADD_SUBDIRECTORY(python)
	ENDIF (HAVE_CYTHON)
ENDIF (HAVE_SERVER)

IF (HAVE_REST)
	ADD_SUBDIRECTORY(events)
ENDIF (HAVE_REST)

# ----------------------------------------
# The SQL persistence module

IF (HAVE_PERSIST_SQL)
	ADD_LIBRARY (PersistModule SHARED
		PersistModule.cc
	)

	TARGET_LINK_LIBRARIES(PersistModule
		# server  # needed for definition of _ZTIN7opencog7RequestE
		${ATOMSPACE_LIBRARIES}
	)

	INSTALL (TARGETS PersistModule
		LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog/modules"
	)
ENDIF (HAVE_PERSIST_SQL)

# ----------------------------------------
# The ZeroMQ persistence module

IF (HAVE_ZMQ AND HAVE_PROTOBUF)

	ADD_LIBRARY (PersistZmqModule SHARED
		PersistZmqModule.cc
	)

	TARGET_LINK_LIBRARIES(PersistZmqModule
		${ATOMSPACE_LIBRARIES} ${PROTOBUF_LIBRARY}
	)

	INSTALL (TARGETS PersistZmqModule
		LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog/modules"
	)

ENDIF (HAVE_ZMQ AND HAVE_PROTOBUF)

# ----------------------------------------
