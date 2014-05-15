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
