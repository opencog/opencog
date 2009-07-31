FIND_PACKAGE(EXPAT REQUIRED)

ADD_LIBRARY(xml SHARED
        FileXMLBufferReader
        NMXmlExporter
        NMXmlParser
        StringXMLBufferReader
)

ADD_DEPENDENCIES(xml opencog_atom_types)

INCLUDE_DIRECTORIES (
	${EXPAT_INCLUDE_DIRS}
)

TARGET_LINK_LIBRARIES(xml
	${EXPAT_LIBRARIES}
)

INSTALL (TARGETS xml
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	FileXMLBufferReader.h
	NMXmlDefinitions.h
	NMXmlExporter.h
	NMXmlParser.h
	StringXMLBufferReader.h
	XMLBufferReader.h
	DESTINATION "include/opencog/persist/xml"
)

TARGET_LINK_LIBRARIES(xml
  ${EXPAT_LIBRARIES}
)
