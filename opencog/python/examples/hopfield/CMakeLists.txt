include_directories(${PROJECT_SOURCE_DIR}/examples/hopfield)
add_library(hopfield_lib SHARED
    ${PROJECT_SOURCE_DIR}/examples/hopfield/HopfieldServer.cc
    ${PROJECT_SOURCE_DIR}/examples/hopfield/Pattern.cc
    ${PROJECT_SOURCE_DIR}/examples/hopfield/HopfieldOptions.cc
    ${PROJECT_SOURCE_DIR}/examples/hopfield/HopfieldUbigrapher.cc
    ${PROJECT_SOURCE_DIR}/examples/hopfield/StorkeyAgent.cc
    ${PROJECT_SOURCE_DIR}/examples/hopfield/ImprintAgent.cc
)

add_library(hopfield_py SHARED
    hopfield_module_wrap.cc
    HopfieldServer_wrap.cc
    HopfieldOptions_wrap.cc
)
target_link_libraries(hopfield_py
    hopfield_lib
    ${Boost_PYTHON}
)
set_target_properties(hopfield_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "hopfield"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/python
)
