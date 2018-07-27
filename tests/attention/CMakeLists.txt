LINK_LIBRARIES (
	server
	attention-types
	attention
	${GSL_LIBRARIES}
	${ATOMSPACE_LIBRARIES}
	${Boost_SYSTEM_LIBRARY}
)

ADD_CXXTEST(AttentionParamQueryUTest)
ADD_CXXTEST(HebbianCreationModuleUTest)
