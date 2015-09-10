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
# For example, since most things depends on the atomspace,
# its is tested first.
IF (CXXTEST_FOUND)

	IF (HAVE_ATOMSPACE)
		# Persistence is for saving/restoring atomspace to disk.
		ADD_SUBDIRECTORY (persist)

		# The cogserver exposes the atomspace to outside users.
		IF (HAVE_SERVER)
			ADD_SUBDIRECTORY (server)
		ENDIF (HAVE_SERVER)

 		ADD_SUBDIRECTORY (spatial)

		ADD_SUBDIRECTORY (nlp)

		ADD_SUBDIRECTORY (pln)

		ADD_SUBDIRECTORY (reasoning)

	ENDIF (HAVE_ATOMSPACE)

	IF (HAVE_CYTHON AND PYTHONINTERP_FOUND)
		ADD_SUBDIRECTORY (cython)
		ADD_SUBDIRECTORY (python)
	ENDIF (HAVE_CYTHON AND PYTHONINTERP_FOUND)

	IF (HAVE_ATTENTION)
		ADD_SUBDIRECTORY (dynamics)
	ENDIF (HAVE_ATTENTION)

	ADD_SUBDIRECTORY (learning)

	# Embodiment requires that learning, spatial, etc.
	# all works fine. So run this last.
	IF (HAVE_EMBODIMENT)
		ADD_SUBDIRECTORY (embodiment)
	ENDIF (HAVE_EMBODIMENT)

ENDIF (CXXTEST_FOUND)
