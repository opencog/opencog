ADD_LIBRARY (dimensionalembedding SHARED
	DimEmbedModule
    CoverTreePoint
)

INCLUDE_DIRECTORIES (
	${ODBC_INCLUDE_DIRS}
)

INSTALL (TARGETS dimensionalembedding
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

TARGET_LINK_LIBRARIES (dimensionalembedding
        util 
)
