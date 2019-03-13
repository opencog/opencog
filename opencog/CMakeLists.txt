
# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

# The build order used here is loosely in terms of
# dependencies: the later parts depend on, or may
# someday depend on the earlier parts.
#
ADD_SUBDIRECTORY (attentionbank)
ADD_SUBDIRECTORY (neighbors)
ADD_SUBDIRECTORY (learning)

IF (HAVE_ATOMSPACE)
	ADD_SUBDIRECTORY (nlp)
	ADD_SUBDIRECTORY (spacetime)
ENDIF (HAVE_ATOMSPACE)

IF (HAVE_SERVER)
	ADD_SUBDIRECTORY (cogserver)
	# attention builds *after* cogserver.
	ADD_SUBDIRECTORY (attention)
ENDIF(HAVE_SERVER)

IF (HAVE_ATOMSPACE AND HAVE_GUILE)
	ADD_SUBDIRECTORY (scm)
	ADD_SUBDIRECTORY (openpsi)
	ADD_SUBDIRECTORY (pln)
	ADD_SUBDIRECTORY (eva)
	ADD_SUBDIRECTORY (ghost)
ENDIF (HAVE_ATOMSPACE AND HAVE_GUILE)

IF (HAVE_CYTHON)
	ADD_SUBDIRECTORY (cython)
	ADD_SUBDIRECTORY (python)
ENDIF (HAVE_CYTHON)

# Graphics and visualization subsystems
ADD_SUBDIRECTORY (visualization)
