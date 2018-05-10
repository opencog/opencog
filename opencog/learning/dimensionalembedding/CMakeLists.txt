ADD_LIBRARY (dimensionalembedding SHARED
	DimEmbedModule
	CoverTreePoint
)

IF (CYGWIN)
	INSTALL (TARGETS dimensionalembedding
		DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
	)
ELSE (CYGWIN)
	INSTALL (TARGETS dimensionalembedding
		LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
	)
ENDIF (CYGWIN)

TARGET_LINK_LIBRARIES (dimensionalembedding
	server
	${ATOMSPACE_LIBRARIES}
	${COGUTIL_LIBRARY}
)
