FIND_PACKAGE(EXPAT REQUIRED)

ADD_LIBRARY(xml
	FileXMLBufferReader.cc
	NMXmlExporter.cc
	NMXmlParser.cc
	StringXMLBufferReader.cc
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/util
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${EXPAT_INCLUDE_DIRS}
)
