include_directories("/usr/include/python2.6")
add_library(hello_py SHARED
    hello.cc
    hello_wrap.cc
)
target_link_libraries(hello_py
    ${Boost_PYTHON}
)
set_target_properties(hello_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "hello"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/opencog/python
)
