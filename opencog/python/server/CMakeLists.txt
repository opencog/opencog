include_directories(
    ${PROJECT_SOURCE_DIR}/opencog/server
    ${Boost_INCLUDE_DIR}
)

add_library(server_py SHARED
    CogServer_wrap.cc
    Factory_wrap.cc
    Agent_wrap.cc
    BaseServer_wrap.cc
    Registry_wrap.cc
    Request_wrap.cc
    server_module_wrap.cc
)

target_link_libraries(server_py
    ${Boost_LIBRARIES}
    server
)

set_target_properties(server_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "libopencog-server-python"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/opencog/python
)

INSTALL (TARGETS server_py
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)
