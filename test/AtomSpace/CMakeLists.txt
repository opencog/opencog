include_directories(
${OPENCOG_SOURCE_DIR}/src/Util
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/NMXml
)
link_directories(
${OPENCOG_BINARY_DIR}/src/AtomSpace
${OPENCOG_BINARY_DIR}/src/NMXml
)
IF(WIN32) 
link_directories(${OPENCOG_SOURCE_DIR}/lib/win32/xmlparser)
ELSE(WIN32)
link_directories(${OPENCOG_SOURCE_DIR}/lib/linux/xmlparser)
ENDIF(WIN32)

add_definitions(-DATOM_TABLE_TEST_XML_FILE=\\"${CMAKE_CURRENT_SOURCE_DIR}/cw_proxy_1.xml\\")
add_cxxtest(AtomTableUTest)
target_link_libraries(AtomTableUTest NMXml AtomSpace Util xmlparse)
IF(WIN32)
target_link_libraries(AtomTableUTest winmm)
ENDIF(WIN32)
       
add_cxxtest(ClassServerUTest)
target_link_libraries(ClassServerUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(ClassServerUTest winmm)
ENDIF(WIN32)
          
add_cxxtest(HandleMapUTest)
target_link_libraries(HandleMapUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(HandleMapUTest winmm)
ENDIF(WIN32)
             
add_cxxtest(NodeUTest)
target_link_libraries(NodeUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(NodeUTest winmm)
ENDIF(WIN32)

add_cxxtest(AtomUTest)
target_link_libraries(AtomUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(AtomUTest winmm)
ENDIF(WIN32)
            
add_cxxtest(CompositeTruthValueUTest)
target_link_libraries(CompositeTruthValueUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(CompositeTruthValueUTest winmm)
ENDIF(WIN32)
  
add_cxxtest(IndefiniteTruthValueUTest)
target_link_libraries(IndefiniteTruthValueUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(IndefiniteTruthValueUTest winmm)
ENDIF(WIN32)
  
add_cxxtest(SimpleTruthValueUTest)
target_link_libraries(SimpleTruthValueUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(SimpleTruthValueUTest winmm)
ENDIF(WIN32)

add_cxxtest(AttentionValueUTest)
target_link_libraries(AttentionValueUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(AttentionValueUTest winmm)
ENDIF(WIN32)
            
add_cxxtest(EconomicAttentionValueUTest)
target_link_libraries(EconomicAttentionValueUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(EconomicAttentionValueUTest winmm)
ENDIF(WIN32)

add_cxxtest(HandleEntryUTest)
target_link_libraries(HandleEntryUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(HandleEntryUTest winmm)
ENDIF(WIN32)

add_cxxtest(LinkUTest)
target_link_libraries(LinkUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(LinkUTest winmm)
ENDIF(WIN32)
                  
add_cxxtest(TruthValueUTest)
target_link_libraries(TruthValueUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(TruthValueUTest winmm)
ENDIF(WIN32)

add_cxxtest(TemporalUTest)
target_link_libraries(TemporalUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(TemporalUTest winmm)
ENDIF(WIN32)

add_cxxtest(HandleSetUTest)
target_link_libraries(HandleSetUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(HandleSetUTest winmm)
ENDIF(WIN32)

add_cxxtest(TemporalMapUTest)
target_link_libraries(TemporalMapUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(TemporalMapUTest winmm)
ENDIF(WIN32)

add_cxxtest(TemporalTableUTest)
target_link_libraries(TemporalTableUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(TemporalTableUTest winmm)
ENDIF(WIN32)

add_cxxtest(AtomSpaceUTest)
target_link_libraries(AtomSpaceUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(AtomSpaceUTest winmm)
ENDIF(WIN32)

add_cxxtest(TimeServerUTest)
target_link_libraries(TimeServerUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(TimeServerUTest winmm)
ENDIF(WIN32)

add_cxxtest(SavingLoadingUTest)
target_link_libraries(SavingLoadingUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(SavingLoadingUTest winmm)
ENDIF(WIN32)

add_cxxtest(utilsUTest)
target_link_libraries(utilsUTest AtomSpace Util)
IF(WIN32)
target_link_libraries(utilsUTest winmm)
ENDIF(WIN32)

