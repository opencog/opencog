ADD_CUSTOM_TARGET(LojbanLib
    COMMAND bash build.sh "${CMAKE_CURRENT_BINARY_DIR}"
    BYPRODUCTS libopencog-lojban-wrapper-0.1.0.0.so
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building Lojban Parser/Printer Library"
)

INSTALL (FILES
    ${CMAKE_CURRENT_BINARY_DIR}/libopencog-lojban-wrapper-0.1.0.0.so
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)
