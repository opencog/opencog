INCLUDE_DIRECTORIES (
	${ODBC_INCLUDE_DIRS}
)

ADD_LIBRARY (samplemodule SHARED
	SampleAgent
)

ADD_LIBRARY (singleagentmodule SHARED
	SingleAgentModule
)
