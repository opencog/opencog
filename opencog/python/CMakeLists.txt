include_directories("/usr/include/python2.6")

# Submodules
add_subdirectory(atomspace)
add_subdirectory(server)

# 'opencog' module
include_directories(./atomspace)
include_directories(./server)
add_library(opencog_py SHARED
    opencog_wrap.cc
)
target_link_libraries(opencog_py
    atomspace_py
    boost_python
    server_py
)
set_target_properties(opencog_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "opencog"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/python
)

# Examples.
add_subdirectory(examples)
