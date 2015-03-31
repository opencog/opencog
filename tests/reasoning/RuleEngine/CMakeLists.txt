
LINK_LIBRARIES(
	ruleengine
	query
	server
	atomspace
)

IF (HAVE_GUILE)
	ADD_CXXTEST(RuleUTest)
	ADD_CXXTEST(ForwardChainerUTest)
	#ADD_CXXTEST(BackwardChainerUTest)
	ADD_CXXTEST(PLNCommonsUTest)
	ADD_CXXTEST(JsonicControlPolicyParamLoaderUTest)
	ADD_CXXTEST(DefaultForwardChainerCBUTest)
	ADD_CXXTEST(FCMemoryUTest)
ENDIF (HAVE_GUILE)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/tests/reasoning/RuleEngine/test_cpolicy.json
               ${PROJECT_BINARY_DIR}/tests/reasoning/RuleEngine/test_cpolicy.json)
