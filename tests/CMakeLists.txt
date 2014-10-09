ENABLE_TESTING()
INCLUDE(AddCxxtest)

IF (WIN32)
	ADD_DEFINITIONS(-D_CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES
	                -D_CRT_SECURE_NO_WARNINGS)
ENDIF (WIN32)

ADD_DEFINITIONS(-DPROJECT_SOURCE_DIR=\\"${CMAKE_SOURCE_DIR}\\"
                -DPROJECT_BINARY_DIR=\\"${CMAKE_BINARY_DIR}\\")

# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

# Perform tests in component-dependency order, as much as possible.
# For example, since everything depends on util and config functions,
# these are tested first. Since most things depend on the atomspace,
# that is tested next, etc.
IF (CXXTEST_FOUND)
	ADD_SUBDIRECTORY (util)

	IF (HAVE_ATOMSPACE)
		ADD_SUBDIRECTORY (atomspace)

		# Persistence is for saving/restoring atomspace to disk.
		IF(HAVE_PERSIST)
			ADD_SUBDIRECTORY (persist)
		ENDIF(HAVE_PERSIST)

		# guile provides scheme bindings for the atomspace.
		IF (HAVE_GUILE)
			ADD_SUBDIRECTORY (scm)
		ENDIF (HAVE_GUILE)

		# Many of the query tests require a working guile, so run
		# these only after guile has been tested.
		ADD_SUBDIRECTORY (query)

		# Currently, reasoning depends on query, above.
		ADD_SUBDIRECTORY (reasoning)

		# The cogserver exposes the atomspace to outside users.
		IF (HAVE_SERVER)
			ADD_SUBDIRECTORY (server)
		ENDIF (HAVE_SERVER)

 		ADD_SUBDIRECTORY (spatial)

		IF (HAVE_VITERBI)
			# Disable until new interfaces widely available. -- linas
			# ADD_SUBDIRECTORY (viterbi)
		ENDIF (HAVE_VITERBI)
	ENDIF (HAVE_ATOMSPACE)

	IF (HAVE_CYTHON AND PYTHONINTERP_FOUND)
		ADD_SUBDIRECTORY (cython)
		ADD_SUBDIRECTORY (python)
	ENDIF (HAVE_CYTHON AND PYTHONINTERP_FOUND)

	IF (HAVE_ATTENTION)
		ADD_SUBDIRECTORY (dynamics)
	ENDIF (HAVE_ATTENTION)

	# Currently, does not depend on atomspace...
	ADD_SUBDIRECTORY (comboreduct)

	# learning requires a working comboreduct.
	ADD_SUBDIRECTORY (learning)

	# Embodiment requires that learning, spatial, etc.
	# all works fine. So run this last.
	IF (HAVE_EMBODIMENT)
		ADD_SUBDIRECTORY (embodiment)
	ENDIF (HAVE_EMBODIMENT)

ENDIF (CXXTEST_FOUND)
