include_directories(${PROJECT_SOURCE_DIR}/opencog/examples/hopfield)
add_library(hopfield_py SHARED
    hopfield_module_wrap.cc
    #HopfieldServer_wrap.cc
)
target_link_libraries(hopfield_py
    server
    attention
    boost_python
)
set_target_properties(hopfield_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "hopfield"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/python
)
