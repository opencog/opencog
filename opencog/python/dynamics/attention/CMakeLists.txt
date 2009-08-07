include_directories(
    ${PROJECT_SOURCE_DIR}/opencog/dynamics/attention
    ${Boost_INCLUDE_DIR}
)

add_library(attention_py SHARED
    attention_module_wrap.cc
    ForgettingAgent_wrap.cc
)

target_link_libraries(attention_py
    attention
    ${Boost_LIBRARIES}
)

set_target_properties(attention_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "libopencog-attention-python"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/opencog/python
)

INSTALL (TARGETS attention_py
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
) 
