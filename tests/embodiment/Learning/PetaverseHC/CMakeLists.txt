ADD_CXXTEST(NeighborhoodGeneratorUTest)
TARGET_LINK_LIBRARIES(NeighborhoodGeneratorUTest
	RewritingRules
	comboreduct
	${COGUTIL_LIBRARY}
)
