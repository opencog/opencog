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

IF (WIN32)
    INSTALL (TARGETS xml DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ELSE (WIN32)
    INSTALL (TARGETS xml LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
ENDIF (WIN32)

INSTALL (FILES
    FileXMLBufferReader.h
    NMXmlDefinitions.h
    NMXmlExporter.h
    NMXmlParser.h
    StringXMLBufferReader.h
    XMLBufferReader.h
    DESTINATION "include/${PROJECT_NAME}/persist/xml"
)

TARGET_LINK_LIBRARIES(xml
    ${EXPAT_LIBRARIES}
)
