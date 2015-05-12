#Create directories
FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/opencong/reasoning ${PROJECT_BINARY_DIR}/opencong/reasoning/pln)

#Copy pln rules to build dir
FILE(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rules DESTINATION ${PROJECT_BINARY_DIR}/opencog/reasoning/pln)

#Copy R2L rules to build dir
FILE(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rules DESTINATION ${PROJECT_BINARY_DIR}/opencog/reasoning/r2l)