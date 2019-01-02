# Used for configuring experimental setup. Run 'make run-ull' from build
# directory to configure the setup for expirmenting with code in run-poc.
ADD_CUSTOM_TARGET(run-ull
  COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/run-poc"
    "${PROJECT_BINARY_DIR}/run-ull"
  COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/run/nonbreaking_prefixes/"
    "${PROJECT_BINARY_DIR}/run-ull/nonbreaking_prefixes/"
  WORKING_DIRECTORY ${CURRENT_SOURCE_DIR}
  COMMENT "Setting run-ull in ${PROJECT_BINARY_DIR}/run-ull"
)


ADD_SUBDIRECTORY (scm)
