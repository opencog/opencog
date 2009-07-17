include_directories(${PROJECT_SOURCE_DIR}/opencog/dynamics/attention)
add_library(attention_py SHARED
    attention_module_wrap.cc
    ForgettingAgent_wrap.cc
)
target_link_libraries(attention_py
    attention
    boost_python
)
set_target_properties(attention_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "attention"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/python
)
