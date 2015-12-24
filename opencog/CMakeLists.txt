
# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

# The build order used here is loosely in terms of
# dependencies: the later parts depend on, or may
# someday depend on the earlier parts.
#
ADD_SUBDIRECTORY (learning)

IF (HAVE_ATOMSPACE)
	ADD_SUBDIRECTORY (nlp)
	ADD_SUBDIRECTORY (planning)
ENDIF (HAVE_ATOMSPACE)

IF (HAVE_SERVER)
	ADD_SUBDIRECTORY (cogserver)
	# attention builds *after* cogserver.
	ADD_SUBDIRECTORY (attention)
	# Octomap checking will be done inside
	# Will always build types
	ADD_SUBDIRECTORY (spacetime)
	IF (HAVE_OCTOMAP)
		ADD_SUBDIRECTORY (spatial)
	ENDIF (HAVE_OCTOMAP)
ENDIF(HAVE_SERVER)

IF (HAVE_ATOMSPACE AND HAVE_GUILE)
	ADD_SUBDIRECTORY (scm)
	ADD_SUBDIRECTORY (openpsi)
ENDIF (HAVE_ATOMSPACE AND HAVE_GUILE)

IF (HAVE_CYTHON)
	ADD_SUBDIRECTORY (cython)
ENDIF (HAVE_CYTHON)

ADD_SUBDIRECTORY (embodiment)

# Graphics and visualization subsystems
ADD_SUBDIRECTORY (visualization)
