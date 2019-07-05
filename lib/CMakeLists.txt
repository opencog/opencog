CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/lib/opencog-chatbot.conf
    ${PROJECT_BINARY_DIR}/lib/opencog-chatbot.conf)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/lib/opencog.conf
    ${PROJECT_BINARY_DIR}/lib/opencog.conf)

# Create a test configuration file by copying the default configuration file
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/lib/opencog-test.conf
    ${PROJECT_BINARY_DIR}/lib/opencog-test.conf)

# Place a default config file in a default location
# CONFDIR normally expands to /usr/local/etc/
# XXX: why is cogserver.conf installed if the default file name is opencog.conf?
INSTALL (FILES
	cogserver.conf
	opencog.conf
	DESTINATION "${CONFDIR}"
)
