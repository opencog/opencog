# opencog.dynamics.attention
add_subdirectory(attention)

# opencog.dynamics
include_directories(./attention)
add_library(dynamics_py SHARED
    dynamics_module_wrap.cc
)
target_link_libraries(dynamics_py
    boost_python
    attention_py
)
set_target_properties(dynamics_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "dynamics"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/python
)
