include_directories(
${OPENCOG_SOURCE_DIR}/src/Util
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/Spatial
${OPENCOG_SOURCE_DIR}/src/NMXml
${OPENCOG_SOURCE_DIR}/test/NMXml
)

link_directories(${OPENCOG_BINARY_DIR})
IF(WIN32)
link_directories(${OPENCOG_SOURCE_DIR}/lib/win32/xmlparser)
ELSE(WIN32)
link_directories(${OPENCOG_SOURCE_DIR}/lib/linux/xmlparser)
ENDIF(WIN32)

add_cxxtest(NMXmlParserUTest NMXmlParserExperiment.cc)
target_link_libraries(NMXmlParserUTest NMXml AtomSpace Util xmlparse)
IF(WIN32)
target_link_libraries(NMXmlParserUTest winmm)
ENDIF(WIN32)

add_cxxtest(NMXmlExporterUTest NMXmlParserExperiment.cc)
target_link_libraries(NMXmlExporterUTest NMXml AtomSpace Util xmlparse)
IF(WIN32)
target_link_libraries(NMXmlExporterUTest winmm)
ENDIF(WIN32)

