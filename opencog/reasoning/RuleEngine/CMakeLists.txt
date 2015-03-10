#rule engine config
ADD_SUBDIRECTORY(rule-engine-src)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/opencog/reasoning/RuleEngine/default_cpolicy.json
	${PROJECT_BINARY_DIR}/opencog/reasoning/RuleEngine/default_cpolicy.json)
