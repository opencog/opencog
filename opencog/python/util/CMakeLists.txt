include_directories(${PROJECT_SOURCE_DIR}/opencog/util)
add_library(util_py SHARED
    util_module_wrap.cc
    Logger_wrap.cc
)
target_link_libraries(util_py
    boost_python
    util
)
set_target_properties(util_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "python_util"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/opencog/python
)
INSTALL (TARGETS util_py
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)
