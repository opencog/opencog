
LINK_LIBRARIES(
	AtomSpaceExtensions
	spacetime
	server
	atomspace
	${Boost_SIGNALS_LIBRARY}
)

ADD_CXXTEST(AtomSpaceUtilUTest)
