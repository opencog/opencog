# Copyright (c) 2008, OpenCog.org (http://opencog.org)
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# The following values are defined
# CXXTEST_FOUND - system has cxxtest
# CXXTEST_GEN   - the binary used to generate the tests

# Find path to the cxxtestgen.py script (NB: this stuff should move to FindCXXTEST.cmake)
# CXXTEST_BIN_DIR enviroment variable must have been defined already

# cxxtest has a Python version and a Perl version. First, look
# for the Python version.
FIND_PACKAGE(PythonInterp)
IF (PYTHONINTERP_FOUND)
	# Look for the .py version first.  Some versions of cxxtest have a
	# non-py version, but it crashes and burns, so, use the .py version
	# preferentially, in order to maintain compatibility across (all??)
	# versions of cxxtest
	FIND_PATH(CXXTEST_PYTHON_BIN_DIR cxxtestgen.py
		$ENV{CXXTEST_BIN_DIR}
		/usr/bin
		/usr/local/bin
		DOC "Where is cxxtest located?"
	)
	IF (CXXTEST_PYTHON_BIN_DIR)
		SET(CXXTEST_FOUND 1)
		SET(CXXTEST_GEN "${CXXTEST_PYTHON_BIN_DIR}/cxxtestgen.py" CACHE FILEPATH "CxxTest binary filepath")
	ENDIF (CXXTEST_PYTHON_BIN_DIR)

	# Sometimes, the python version doesn't have a .py extension.
	# repeat the above search, without the extension.
	IF (NOT CXXTEST_FOUND)
		FIND_PATH(CXXTEST_PYTHON_BIN_DIR cxxtestgen
			$ENV{CXXTEST_BIN_DIR}
			/usr/bin
			/usr/local/bin
			DOC "Where is cxxtest located?"
		)
		IF (CXXTEST_PYTHON_BIN_DIR)
			SET(CXXTEST_FOUND 1)
			SET(CXXTEST_GEN "${CXXTEST_PYTHON_BIN_DIR}/cxxtestgen" CACHE FILEPATH "CxxTest binary filepath")
		ENDIF (CXXTEST_PYTHON_BIN_DIR)
	ENDIF (NOT CXXTEST_FOUND)
ENDIF (PYTHONINTERP_FOUND)

# If we still haven't found it, try the perl version.
IF (NOT CXXTEST_FOUND)
	# The python version wasn't found--search for the perl version.
	FIND_PATH(CXXTEST_PERL_BIN_DIR cxxtestgen.pl
		$ENV{CXXTEST_BIN_DIR}
		/usr/bin
		/usr/local/bin
		DOC "Where is cxxtest located?"
	)
	IF (CXXTEST_PERL_BIN_DIR)
		SET(CXXTEST_FOUND 1)
		SET(CXXTEST_GEN "${CXXTEST_PERL_BIN_DIR}/cxxtestgen.pl" CACHE FILEPATH "CxxTest binary filepath")
	ENDIF (CXXTEST_PERL_BIN_DIR)
ENDIF (NOT CXXTEST_FOUND)


IF (WIN32)
	FIND_PATH(CXXTEST_INCLUDE_DIR "TestSuite.h"
		$ENV{CXXTEST_INCLUDE_DIR}
		"C:/Program Files/Cxxtest/cxxtest"
		"C:/Cxxtest/cxxtest"
		DOC "Where are cxxtest include files?"
	)
	IF (CXXTEST_BIN_DIR AND CXXTEST_INCLUDE_DIR)
	   GET_FILENAME_COMPONENT(CXXTEST_INCLUDE_DIRS "${CXXTEST_INCLUDE_DIR}" PATH)
	ELSE (CXXTEST_BIN_DIR AND CXXTEST_INCLUDE_DIR)
	   SET(CXXTEST_INCLUDE_DIRS)
	   SET(CXXTEST_FOUND 0)
	ENDIF (CXXTEST_BIN_DIR AND CXXTEST_INCLUDE_DIR)
ENDIF (WIN32)

# abort if required the results.
IF(NOT CXXTEST_FOUND)
	SET(CXXTEST_BIN_DIR_MESSAGE "Cxxtest was not found. Make sure CXXTEST_BIN_DIR is set.")
	IF(NOT Cxxtest_FIND_QUIETLY)
		MESSAGE(STATUS "${CXXTEST_BIN_DIR_MESSAGE}")
	ELSE(NOT Cxxtest_FIND_QUIETLY)
		IF(Cxxtest_FIND_REQUIRED)
			MESSAGE(FATAL_ERROR "${CXXTEST_BIN_DIR_MESSAGE}")
		ENDIF(Cxxtest_FIND_REQUIRED)
	ENDIF(NOT Cxxtest_FIND_QUIETLY)
ENDIF(NOT CXXTEST_FOUND)

# Leaving this set interfers with the Python3 search.
UNSET(PYTHONINTERP_FOUND)
