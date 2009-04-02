INCLUDE_DIRECTORIES(
	${OPENCOG_SOURCE_DIR}/opencog
	${OPENCOG_SOURCE_DIR}/opencog/util
	${OPENCOG_SOURCE_DIR}/opencog/atomspace
	${OPENCOG_SOURCE_DIR}/opencog/xml
        ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
)

LINK_DIRECTORIES(
	${OPENCOG_BINARY_DIR}/opencog/atomspace
	${OPENCOG_BINARY_DIR}/opencog/xml
        ${OPENCOG_BINARY_DIR}/opencog/embodiment/AtomSpaceExtensions
)

LINK_LIBRARIES(
	atomspace
	${Boost_SIGNALS_LIBRARY}
        AtomSpaceExtensions
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
