# Build file for the atomspace benchmark

IF (HAVE_CYTHON)
	INCLUDE_DIRECTORIES (
		${PYTHON_INCLUDE_PATH}
	)
ENDIF (HAVE_CYTHON)

ADD_EXECUTABLE (atomspace_bm
	AtomSpaceBenchmark
	atomspace_bm
)

IF (HAVE_GUILE)
	TARGET_LINK_LIBRARIES (atomspace_bm smob server)
ENDIF(HAVE_GUILE)

IF (HAVE_CYTHON)
	TARGET_LINK_LIBRARIES (atomspace_bm PythonModule atomspace_cython)
ENDIF(HAVE_CYTHON)

TARGET_LINK_LIBRARIES (atomspace_bm
	server
	atomspace
	cogutil
)
