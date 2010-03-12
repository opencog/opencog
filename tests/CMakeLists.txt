ENABLE_TESTING()
INCLUDE(AddCxxtest)

IF (WIN32)
	ADD_DEFINITIONS(-D_CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES
                   -D_CRT_SECURE_NO_WARNINGS)
ENDIF (WIN32)

ADD_DEFINITIONS(-DOPENCOG_SOURCE_DIR=\\"${CMAKE_SOURCE_DIR}\\"
                -DOPENCOG_BINARY_DIR=\\"${CMAKE_BINARY_DIR}\\")

# Perform tests in component-dependency order, as much as possible.
# For example, since everything depends on util and config functions,
# these are tested first. Since most things depend on the atomspace,
# that is tested next, etc.
IF (CXXTEST_FOUND)
	ADD_SUBDIRECTORY (util)
	ADD_SUBDIRECTORY (atomspace)
	ADD_SUBDIRECTORY (persist)

	IF (GUILE_FOUND)
		ADD_SUBDIRECTORY (scm)
	ENDIF (GUILE_FOUND)

	ADD_SUBDIRECTORY (query)
	ADD_SUBDIRECTORY (server)
ENDIF (CXXTEST_FOUND)

# Rest interfaces assume that the cogserver is working;
# so run after the cogserver tests.
ADD_SUBDIRECTORY (web)

IF (CXXTEST_FOUND)
	ADD_SUBDIRECTORY (dynamics)
	ADD_SUBDIRECTORY (reasoning)
	ADD_SUBDIRECTORY (comboreduct)
	ADD_SUBDIRECTORY (spatial)

	IF (XERCESC_FOUND AND LUABIND_FOUND AND EXPAT_FOUND)
		ADD_SUBDIRECTORY (embodiment)
	ENDIF (XERCESC_FOUND AND LUABIND_FOUND AND EXPAT_FOUND)

	IF (EXPAT_FOUND)
		ADD_SUBDIRECTORY (learning)
	ENDIF (EXPAT_FOUND)
ENDIF (CXXTEST_FOUND)

