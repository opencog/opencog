
# The atom_types.h file is written to the build directory
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

DECLARE_GUILE_CONFIG_TARGET(SCM_CONFIG "opencog oc-config" "OPENCOG_TEST")

# The build order used here is loosely in terms of
# dependencies: the later parts depend on, or may
# someday depend on the earlier parts.
#
ADD_SUBDIRECTORY (neighbors)

IF (HAVE_ATOMSPACE)
	ADD_SUBDIRECTORY (nlp)
ENDIF (HAVE_ATOMSPACE)

IF (HAVE_ATOMSPACE AND HAVE_GUILE)
	ADD_SUBDIRECTORY (eva)
ENDIF (HAVE_ATOMSPACE AND HAVE_GUILE)

IF (HAVE_OPENPSI)
	ADD_SUBDIRECTORY (openpsi)
ENDIF (HAVE_OPENPSI)

IF (HAVE_NLP)
	ADD_SUBDIRECTORY (ghost)
ENDIF (HAVE_NLP)

IF (HAVE_PLN)
	ADD_SUBDIRECTORY (pln)
ENDIF (HAVE_PLN)

IF (HAVE_CYTHON)
	ADD_SUBDIRECTORY (cython)
	ADD_SUBDIRECTORY (python)
ENDIF (HAVE_CYTHON)

WRITE_GUILE_CONFIG(${GUILE_BIN_DIR}/opencog/oc-config.scm SCM_CONFIG TRUE)

WRITE_GUILE_CONFIG(${GUILE_BIN_DIR}/opencog/oc-config-installable.scm SCM_CONFIG FALSE)
INSTALL(FILES ${GUILE_BIN_DIR}/opencog/oc-config-installable.scm DESTINATION ${GUILE_SITE_DIR}/opencog RENAME oc-config.scm)
