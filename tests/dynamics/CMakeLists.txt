ADD_SUBDIRECTORY (scm)

LINK_LIBRARIES (
	server
	attention-types
	attention
	${GSL_LIBRARIES}
)

ADD_CXXTEST(ForgettingAgentUTest)
ADD_CXXTEST(HebbianUpdatingAgentUTest)
ADD_CXXTEST(ImportanceUpdatingAgentUTest)
IF (GSL_FOUND)
	ADD_CXXTEST(ImportanceDiffusionAgentUTest)
ENDIF (GSL_FOUND)
ADD_CXXTEST(HebbianCreationModuleUTest)
ADD_CXXTEST(SimpleImportanceDiffusionAgentUTest)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/dynamics/scm/example-1.scm
    ${PROJECT_BINARY_DIR}/tests/dynamics/scm/example-1.scm)
