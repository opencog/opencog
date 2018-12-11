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
        ADD_SUBDIRECTORY (attention)

        # Persistence is for saving/restoring atomspace to disk.
        ADD_SUBDIRECTORY (persist)

        # The cogserver exposes the atomspace to outside users.
        IF (HAVE_SERVER)
            ADD_SUBDIRECTORY (server)
        ENDIF (HAVE_SERVER)

        IF (HAVE_OCTOMAP)
            ADD_SUBDIRECTORY (spacetime)
        ENDIF (HAVE_OCTOMAP)

        ADD_SUBDIRECTORY (nlp)

        ADD_SUBDIRECTORY (pln)

        ADD_SUBDIRECTORY (openpsi)

    ENDIF (HAVE_ATOMSPACE)

    IF (HAVE_CYTHON AND PYTHONINTERP_FOUND)
        ADD_SUBDIRECTORY (cython)
        ADD_SUBDIRECTORY (python)
    ENDIF (HAVE_CYTHON AND PYTHONINTERP_FOUND)

    ADD_SUBDIRECTORY (learning)

ENDIF (CXXTEST_FOUND)

IF (HAVE_GUILE)
    ADD_SUBDIRECTORY(ghost)
ENDIF()
