# Used for configuring experimental setup. Run 'make run-ull' from build
# directory to configure the setup for expirmenting with code in run-poc.
# NOTE: The order of the command matters, so test after making changes.
ADD_CUSTOM_TARGET(run-ull
  COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/run-poc"
    "${PROJECT_BINARY_DIR}/run-ull"
  COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/run/nonbreaking_prefixes/"
    "${PROJECT_BINARY_DIR}/run-ull/nonbreaking_prefixes/"
  COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/run/split-sentences.pl"
    "${PROJECT_BINARY_DIR}/run-ull/"
  COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/run/config/*"
    "${PROJECT_BINARY_DIR}/run-ull/config"
  WORKING_DIRECTORY ${CURRENT_SOURCE_DIR}
  COMMENT "Setting run-ull in ${PROJECT_BINARY_DIR}/run-ull"
)


ADD_SUBDIRECTORY (scm)
