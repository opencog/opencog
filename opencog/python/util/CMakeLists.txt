include_directories(
    ${PROJECT_SOURCE_DIR}/opencog/util
    ${Boost_INCLUDE_DIR}
)

add_library(util_py SHARED
    util_module_wrap.cc
    Logger_wrap.cc
)

target_link_libraries(util_py
    ${Boost_PYTHON_LIBRARY}
    util
)

set_target_properties(util_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "libopencog-util-python"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/opencog/python
)

INSTALL (TARGETS util_py
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)
