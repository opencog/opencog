ADD_SUBDIRECTORY (TimeSpaceAtomMapsAPI)

LINK_LIBRARIES(
	spacetime-types
	spacetime
)

ADD_CXXTEST(SpaceServerUTest)

