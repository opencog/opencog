include_directories("/usr/include/python2.6")
add_library(custom_string_py SHARED
    custom_string.cc
)
target_link_libraries(custom_string_py
    ${Boost_PYTHON}
)
set_target_properties(custom_string_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "custom_string"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/opencog/python
)
