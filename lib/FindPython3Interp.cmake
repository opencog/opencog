# Find the Python3 interpreter.
#
# NB: Generally, :code:`FindPythonInterp.cmake` as shipped by cmake
# is able to find a python3 interpreter by working with version numbers. This find module
# does some gymnastics to be able to find BOTH python2 and python3
# interpreters within the same project, which does not seem possible
# with the modules provided from upstream.
#
#
# When using this to look for Python3, you should use
# FindPython2Interp.cmake to look for the Python2 interpreter.
#
# This module sets the following variables:
#
#  PYTHON3INTERP_FOUND         - Was the Python executable found
#  PYTHON3_EXECUTABLE          - path to the Python interpreter
#
#  PYTHON3_VERSION_STRING      - Python version found e.g. 2.5.2
#  PYTHON3_VERSION_MAJOR       - Python major version found e.g. 2
#  PYTHON3_VERSION_MINOR       - Python minor version found e.g. 5
#  PYTHON3_VERSION_PATCH       - Python patch version found e.g. 2
#

# Nuke the cache, somebody might have looked for Python 2...
if(PYTHONINTERP_FOUND)
  message(WARNING "Please look for Python3 before looking for Python2...")
endif()
unset(PYTHON_EXECUTABLE CACHE)
set(PYTHONINTERP_FOUND FALSE)

find_package(PythonInterp 3 QUIET)
find_package_handle_standard_args(Python3Interp
                                  REQUIRED_VARS PYTHON_EXECUTABLE)

# Set all those variables that we promised
set(PYTHON3_EXECUTABLE ${PYTHON_EXECUTABLE})
set(PYTHON3_VERSION_STRING ${PYTHON_VERSION_STRING})
set(PYTHON3_VERSION_MAJOR ${PYTHON_VERSION_MAJOR})
set(PYTHON3_VERSION_MINOR ${PYTHON_VERSION_MINOR})
set(PYTHON3_VERSION_PATCH ${PYTHON_VERSION_PATCH})

# Now nuke the cache to allow later rerunning of find_package(PythonInterp)
# with a different required version number.
unset(PYTHON_EXECUTABLE CACHE)
set(PYTHONINTERP_FOUND FALSE)
