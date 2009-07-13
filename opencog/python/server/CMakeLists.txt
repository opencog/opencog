include_directories(${PROJECT_SOURCE_DIR}/opencog/server)
add_library(server_py SHARED
    BaseServer_wrap.cc
    CogServer_wrap.cc
    Registry_wrap.cc
    server_module_wrap.cc
)
target_link_libraries(server_py
    boost_python
    server
)
set_target_properties(server_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "server"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/python
)
