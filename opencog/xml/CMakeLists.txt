FIND_PACKAGE(EXPAT REQUIRED)

ADD_LIBRARY(xml
	FileXMLBufferReader.cc
	NMXmlExporter.cc
	NMXmlParser.cc
	StringXMLBufferReader.cc
)

INCLUDE_DIRECTORIES (
	${EXPAT_INCLUDE_DIRS}
)

INSTALL (FILES
	FileXMLBufferReader.h
	NMXmlDefinitions.h
	NMXmlExporter.h
	NMXmlParser.h
	StringXMLBufferReader.h
	XMLBufferReader.h
	DESTINATION "include/opencog/xml"
)
