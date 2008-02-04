SET(CMAKE_BUILD_TYPE Debug)

add_library (NMXml
FileXMLBufferReader.cc  NMXmlExporter.cc  NMXmlParser.cc StringXMLBufferReader.cc
)

include_directories (
${OPENCOG_SOURCE_DIR}/src/Util
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/NMXml
${OPENCOG_SOURCE_DIR}/include/xmlparser
)

