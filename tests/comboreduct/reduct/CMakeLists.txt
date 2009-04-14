#ADD_CXXTEST(ActionReductUTest)
#TARGET_LINK_LIBRARIES(ActionReductUTest
#	comboreduct
#   util
#)

ADD_CXXTEST(BooleanReductUTest)
TARGET_LINK_LIBRARIES(BooleanReductUTest
	comboreduct
    util
)

ADD_CXXTEST(ContinReductUTest)
TARGET_LINK_LIBRARIES(ContinReductUTest
	comboreduct
    util
)

ADD_CXXTEST(FullReductUTest)
TARGET_LINK_LIBRARIES(FullReductUTest
	comboreduct
    util
)

ADD_CXXTEST(MixedReductUTest)
TARGET_LINK_LIBRARIES(MixedReductUTest
	comboreduct
    util
)

#ADD_CXXTEST(PerceptionReductUTest)
#TARGET_LINK_LIBRARIES(PerceptionReductUTest
#	comboreduct
#   util
#)
