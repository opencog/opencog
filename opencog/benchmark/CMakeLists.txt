LINK_DIRECTORIES (
	${PROJECT_BINARY_DIR}/opencog/atomspace
)

#INCLUDE_DIRECTORIES (
#)

ADD_EXECUTABLE (atomspace_bm
	AtomSpaceBenchmark
	atomspace_bm
)

TARGET_LINK_LIBRARIES (atomspace_bm
	atomspace
	cogutil
)
