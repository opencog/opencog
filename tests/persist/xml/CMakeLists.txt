INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/src/util
	${PROJECT_SOURCE_DIR}/src/atomspace
	${PROJECT_SOURCE_DIR}/src/persist/xml
	${PROJECT_SOURCE_DIR}/test/xml
)

LINK_DIRECTORIES(${PROJECT_BINARY_DIR})

LINK_LIBRARIES(atomspace
	xml
	${Boost_SIGNALS_LIBRARY}
	${EXPAT_LIBRARIES}
)

ADD_CXXTEST(NMXmlParserUTest NMXmlParserExperiment.cc)
ADD_CXXTEST(NMXmlExporterUTest NMXmlParserExperiment.cc)
