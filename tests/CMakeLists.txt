ENABLE_TESTING()
INCLUDE(AddCxxtest)

IF (WIN32)
	ADD_DEFINITIONS(-D_CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES
                   -D_CRT_SECURE_NO_WARNINGS)
ENDIF (WIN32)

ADD_DEFINITIONS(-DPROJECT_SOURCE_DIR=\\"${CMAKE_SOURCE_DIR}\\"
                -DPROJECT_BINARY_DIR=\\"${CMAKE_BINARY_DIR}\\")

# Perform tests in component-dependency order, as much as possible.
# For example, since everything depends on util and config functions,
# these are tested first. Since most things depend on the atomspace,
# that is tested next, etc.
IF (CXXTEST_FOUND)
	ADD_SUBDIRECTORY (util)

    IF (HAVE_ATOMSPACE)
   	    ADD_SUBDIRECTORY (atomspace)
        ADD_SUBDIRECTORY (query)
    ENDIF (HAVE_ATOMSPACE)

    IF(HAVE_PERSIST)
        ADD_SUBDIRECTORY (persist)
    ENDIF(HAVE_PERSIST)

	IF (HAVE_GUILE)
		ADD_SUBDIRECTORY (scm)
	ENDIF (HAVE_GUILE)

    IF (HAVE_SERVER)
    	ADD_SUBDIRECTORY (server)
    ENDIF (HAVE_SERVER)

    # Rest interfaces assume that the cogserver is working;
    # so run after the cogserver tests.
    IF(HAVE_WEB)
        ADD_SUBDIRECTORY (web)
    ENDIF(HAVE_WEB)

    IF(HAVE_CYTHON)
        ADD_SUBDIRECTORY (cython)
    ENDIF(HAVE_CYTHON)

    IF(HAVE_ATTENTION)
        ADD_SUBDIRECTORY (dynamics)
    ENDIF(HAVE_ATTENTION)

    IF(HAVE_PLN)
        ADD_SUBDIRECTORY (reasoning)
    ENDIF(HAVE_PLN)

	ADD_SUBDIRECTORY (comboreduct)
    IF(HAVE_SPATIAL)
       	ADD_SUBDIRECTORY (spatial)
    ENDIF(HAVE_SPATIAL)

	IF (HAVE_EMBODIMENT)
		ADD_SUBDIRECTORY (embodiment)
	ENDIF (HAVE_EMBODIMENT)

	ADD_SUBDIRECTORY (learning)
ENDIF (CXXTEST_FOUND)
