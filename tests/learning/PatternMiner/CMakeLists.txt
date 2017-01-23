IF (HAVE_cpprest)
	ADD_CXXTEST(PatternMinerUTest)
	TARGET_LINK_LIBRARIES(PatternMinerUTest
		PatternMiner
	)
ENDIF (HAVE_cpprest)
