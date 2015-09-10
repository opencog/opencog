
LINK_LIBRARIES(
	AtomSpaceExtensions
	spacetime
	server
	${ATOMSPACE_LIBRARY}
)

ADD_CXXTEST(AtomSpaceUtilUTest)
