#
# XXX the correct python version number should be automatically
# determined and modified/set here.  
#
INCLUDE_DIRECTORIES(
    ${PYTHON_INCLUDE_PATH}
    ${Boost_INCLUDE_DIR}
)
#include_directories("/usr/include/python2.6")
#include_directories("/usr/include/python2.5")
#include_directories("/usr/include/python2.4")

# Submodules
add_subdirectory(atomspace)
add_subdirectory(server)
add_subdirectory(dynamics)
add_subdirectory(util)
add_subdirectory(examples)

# 'opencog' module
include_directories(./atomspace)
include_directories(./server)
include_directories(./dynamics)
include_directories(./util)
include_directories(./examples/hopfield)
add_library(opencog_py SHARED
    opencog_module_wrap.cc
)
target_link_libraries(opencog_py
    ${Boost_LIBRARIES}

    atomspace_py
    server_py
    dynamics_py
    util_py
    # hopfield_py
)
set_target_properties(opencog_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "opencog"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/opencog/python
)
INSTALL (TARGETS
    opencog_py
	LIBRARY DESTINATION "${PYTHON_DEST}"
)
