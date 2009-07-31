INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/util
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/persist/xml
	${OPENCOG_SOURCE_DIR}/test/xml
)

LINK_DIRECTORIES(${OPENCOG_BINARY_DIR})

LINK_LIBRARIES(atomspace
	xml
	${Boost_SIGNALS_LIBRARY}
	${EXPAT_LIBRARIES}
)

ADD_CXXTEST(NMXmlParserUTest NMXmlParserExperiment.cc)
ADD_CXXTEST(NMXmlExporterUTest NMXmlParserExperiment.cc)
