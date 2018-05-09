LINK_LIBRARIES(
	query
	execution
	attentionbank
	atomspace
	atomspaceutils
	atombase
	atomutils
)

ADD_CXXTEST(AttentionValueUTest)
ADD_CXXTEST(AttentionUTest)
ADD_CXXTEST(BankImplUTest)
ADD_CXXTEST(BankAsyncUTest)

ADD_CXXTEST(AttentionalFocusCBUTest)
