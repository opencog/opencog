#ADD_CXXTEST(ActionReductUTest)
#TARGET_LINK_LIBRARIES(ActionReductUTest
#	comboreduct
#   util
#)

ADD_CXXTEST(BooleanReductUTest)
TARGET_LINK_LIBRARIES(BooleanReductUTest
	comboreduct
	cogutil
)

ADD_CXXTEST(BooleanFullUTest)
TARGET_LINK_LIBRARIES(BooleanFullUTest
	comboreduct
	cogutil
)

ADD_CXXTEST(ContinReductUTest)
TARGET_LINK_LIBRARIES(ContinReductUTest
	comboreduct
	cogutil
)

ADD_CXXTEST(FullReductUTest)
TARGET_LINK_LIBRARIES(FullReductUTest
	comboreduct
	cogutil
)

ADD_CXXTEST(MixedReductUTest)
TARGET_LINK_LIBRARIES(MixedReductUTest
	comboreduct
	cogutil
)

#ADD_CXXTEST(PerceptionReductUTest)
#TARGET_LINK_LIBRARIES(PerceptionReductUTest
#	comboreduct
#   util
#)

ADD_CXXTEST(ANNReductUTest)
TARGET_LINK_LIBRARIES(ANNReductUTest
	comboreduct
	cogutil
)

ADD_CXXTEST(FoldReductUTest)
TARGET_LINK_LIBRARIES(FoldReductUTest
	comboreduct
	cogutil
)
