include_directories("/usr/include/python2.6")

# Submodules
add_subdirectory(atomspace)
add_subdirectory(server)
add_subdirectory(dynamics)
add_subdirectory(util)

# 'opencog' module
include_directories(./atomspace)
include_directories(./server)
include_directories(./dynamics)
include_directories(./util)
add_library(opencog_py SHARED
    opencog_module_wrap.cc
)
target_link_libraries(opencog_py
    boost_python

    atomspace_py
    server_py
    dynamics_py
    util_py
)
set_target_properties(opencog_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "opencog"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/python
)

# Examples.
add_subdirectory(examples)
