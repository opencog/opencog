LINK_LIBRARIES(
	query
	execution
	atombase
	atomutils
	atomspaceutils
	atomspace
	attentionbank
	clearbox
)

ADD_CXXTEST(AttentionValueUTest)
ADD_CXXTEST(AttentionUTest)
ADD_CXXTEST(AtomSpaceImplUTest)
ADD_CXXTEST(AttentionalFocusCBUTest)
