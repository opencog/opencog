CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/lib/opencog-chatbot.conf
    ${PROJECT_BINARY_DIR}/lib/opencog-chatbot.conf)
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/lib/opencog.conf
    ${PROJECT_BINARY_DIR}/lib/opencog.conf)
# Create a test configuration file by copying the default configuration file
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/lib/opencog-test.conf
    ${PROJECT_BINARY_DIR}/lib/opencog-test.conf)
# Rule engine conf file
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/lib/rule-engine.conf
    ${PROJECT_BINARY_DIR}/lib/rule-engine.conf)
ADD_SUBDIRECTORY (json_spirit)
