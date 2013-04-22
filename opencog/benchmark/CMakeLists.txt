LINK_DIRECTORIES (
	${PROJECT_BINARY_DIR}/opencog/atomspace
)

#INCLUDE_DIRECTORIES (
#)

ADD_EXECUTABLE (atomspace_bm
	AtomSpaceBenchmark
	atomspace_bm
)

IF(HAVE_GUILE)
	TARGET_LINK_LIBRARIES (atomspace_bm smob)
ENDIF(HAVE_GUILE)

TARGET_LINK_LIBRARIES (atomspace_bm
	server
	atomspace
	cogutil
)
