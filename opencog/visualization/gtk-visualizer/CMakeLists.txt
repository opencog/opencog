

ADD_EXECUTABLE(OpenCogAtomSpaceVisualizer
	AtomSpaceInterface
	AtomTypes
	Graph
	main
	Positions
	Vertex
	Vertices
)

INCLUDE_DIRECTORIES(
	${GTK3_INCLUDE_DIRS}
)

TARGET_LINK_LIBRARIES(OpenCogAtomSpaceVisualizer
	${GTK3_LIBRARIES}
	${Boost_SYSTEM_LIBRARY}
)

