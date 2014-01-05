INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/opencog/util
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/persist/xml
	${PROJECT_SOURCE_DIR}/test/xml
)

LINK_DIRECTORIES(${PROJECT_BINARY_DIR})

LINK_LIBRARIES(atomspace
	xml
	${EXPAT_LIBRARIES}
)

ADD_CXXTEST(NMXmlParserUTest NMXmlParserExperiment.cc)
ADD_CXXTEST(NMXmlExporterUTest NMXmlParserExperiment.cc)
