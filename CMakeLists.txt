LINK_LIBRARIES(
	query
	attentionbank
	atomspace
)

ADD_CXXTEST(AttentionValueUTest)
ADD_CXXTEST(AttentionUTest)
ADD_CXXTEST(BankImplUTest)
ADD_CXXTEST(BankAsyncUTest)

ADD_CXXTEST(AttentionalFocusCBUTest)
