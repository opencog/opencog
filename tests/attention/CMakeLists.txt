LINK_LIBRARIES (
	attention-types
	attentionbonk
	attention
	${COGSERVER_LIBRARIES}
	${ATOMSPACE_LIBRARIES}
	${GSL_LIBRARIES}
	${Boost_SYSTEM_LIBRARY}
)

ADD_CXXTEST(AttentionParamQueryUTest)
ADD_CXXTEST(ImportanceDiffusionUTest)
ADD_CXXTEST(HebbianCreationModuleUTest)
