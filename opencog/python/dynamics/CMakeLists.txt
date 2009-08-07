# opencog.dynamics.attention
add_subdirectory(attention)

# opencog.dynamics
include_directories(./attention
    ${Boost_INCLUDE_DIR}
)
add_library(dynamics_py SHARED
    dynamics_module_wrap.cc
)
target_link_libraries(dynamics_py
    ${Boost_LIBRARIES}
    attention_py
)
set_target_properties(dynamics_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "libopencog-dynamics-python"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/opencog/python
)
INSTALL (TARGETS dynamics_py
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)
