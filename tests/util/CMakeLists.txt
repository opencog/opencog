INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/opencog/util
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}
	${PROJECT_BINARY_DIR}/opencog/util
)

LINK_LIBRARIES(
	cogutil
)

ADD_CXXTEST(ConfigUTest)
ADD_CXXTEST(LoggerUTest)
ADD_CXXTEST(StringTokenizerUTest)
ADD_CXXTEST(lazy_selectorUTest)
ADD_CXXTEST(lru_cacheUTest)
ADD_CXXTEST(iostreamContainerUTest)
#ADD_CXXTEST(numericUTest)
ADD_CXXTEST(functionalUTest)
ADD_CXXTEST(KLDUTest)
ADD_CXXTEST(randomUTest)
ADD_CXXTEST(comprehensionUTest)
ADD_CXXTEST(CounterUTest)
ADD_CXXTEST(rankingUTest)
