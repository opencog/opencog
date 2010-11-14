ADD_LIBRARY (dimensionalembedding SHARED
	DimEmbedModule
    CoverTreeNode
)

INCLUDE_DIRECTORIES (
	${ODBC_INCLUDE_DIRS}
)

INSTALL (TARGETS dimensionalembedding
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)
