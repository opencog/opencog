INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/src/util
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/xml
)

LINK_DIRECTORIES(
	${OPENCOG_BINARY_DIR}/src/atomspace
	${OPENCOG_BINARY_DIR}/src/xml
)

LINK_LIBRARIES(
	atomspace
	${Boost_SIGNALS_LIBRARY}
)

# AtomTableUTest requires the XML libraries
IF (EXPAT_FOUND)
	ADD_CXXTEST(AtomTableUTest)
	TARGET_LINK_LIBRARIES(AtomTableUTest xml ${EXPAT_LIBRARIES})
ENDIF (EXPAT_FOUND)

ADD_CXXTEST(ClassServerUTest)
ADD_CXXTEST(HandleMapUTest)
ADD_CXXTEST(NodeUTest)
ADD_CXXTEST(AtomUTest)
ADD_CXXTEST(CompositeTruthValueUTest)
ADD_CXXTEST(IndefiniteTruthValueUTest)
ADD_CXXTEST(SimpleTruthValueUTest)
ADD_CXXTEST(AttentionValueUTest)
ADD_CXXTEST(HandleEntryUTest)
ADD_CXXTEST(LinkUTest)
ADD_CXXTEST(TruthValueUTest)
ADD_CXXTEST(TemporalUTest)
ADD_CXXTEST(HandleSetUTest)
ADD_CXXTEST(TemporalMapUTest)
ADD_CXXTEST(TemporalTableUTest)
ADD_CXXTEST(AtomSpaceUTest)
ADD_CXXTEST(TimeServerUTest)
ADD_CXXTEST(SavingLoadingUTest)
